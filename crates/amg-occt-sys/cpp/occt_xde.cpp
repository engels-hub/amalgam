// STEP and IGES through XCAF (S02 §9, BREP-13). Declarations and categories are in occt_shim.hxx.
//
// Reading fills a private XCAF document (`TDocStd_Document` without an application, length unit
// mm), walks its free shapes into plain nodes (names, locations, colours, prototype indices) and
// keeps one shape per prototype label; the document is dropped before the call returns. Writing
// builds such a document from plain nodes over private copies of the caller's shapes (adding a
// shape to an XCAF assembly puts it into compounds, which clears its TShape's `Free` flag:
// BREP-10's `TopoDS_Builder::Add` finding), then runs the STEP or IGES writer.
//
// STEP reads parse from a stream that reports the bytes read and stops (end of file) once the
// sink asks to, so cancellation reaches the parser; the transfer reports through a
// `Message_ProgressRange`. IGES is not thread-safe in OCCT 8.0: its functions run under
// amg-brep's global lock (category G).

#include "amg-occt-sys/cpp/occt_shim.hxx"

#include "amg-occt-sys/src/bridge.rs.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>

#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <DESTEP_Parameters.hxx>
#include <Geom_Curve.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IGESCAFControl_Reader.hxx>
#include <IGESCAFControl_Writer.hxx>
#include <IGESControl_Controller.hxx>
#include <IGESData_GlobalSection.hxx>
#include <IGESData_IGESModel.hxx>
#include <Interface_Static.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <OSD_OpenFile.hxx>
#include <Quantity_Color.hxx>
#include <Quantity_ColorRGBA.hxx>
#include <STEPCAFControl_Controller.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <StepBasic_Organization.hxx>
#include <StepBasic_Person.hxx>
#include <StepData_StepModel.hxx>
#include <ShapeFix_Edge.hxx>
#include <Standard_ConstructionError.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDF_Tool.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <UnitsMethods_LengthUnit.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_ColorType.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XSControl_WorkSession.hxx>
#include <gp_Trsf.hxx>

namespace amg::occt {

// What a read gave: plain data and one shape per prototype (only read after the reader returns).
struct XdeData {
  OcXdeStatus status;
  std::vector<OcXdeNode> nodes;
  std::vector<OcXdeProto> protos;
  std::vector<OcXdeFaceColor> face_colors;
  std::vector<TopoDS_Shape> shapes;
  // A cancelled STEP read keeps its reader (and the parsed model) here, so the caller can free it
  // off the cancelling thread: tearing down a 50 MB model takes about half a second.
  std::shared_ptr<void> keep;
};

OcXde::OcXde() : data(std::make_unique<XdeData>()) {}
OcXde::~OcXde() = default;

namespace {

constexpr std::uint32_t NONE = UINT32_MAX;

[[noreturn]] void fail(const std::string &message) {
  throw Standard_ConstructionError(message.c_str());
}

// The process-wide registrations of the STEP and IGES translators (static tables), once.
void init_translators() {
  static std::once_flag once;
  std::call_once(once, [] {
    STEPCAFControl_Controller::Init();
    IGESControl_Controller::Init();
    // IGES writes solids as MSBO B-rep entities (type 186), not as loose trimmed surfaces, so
    // they read back as solids. Only amg's IGES writer reads this; it runs under the global lock.
    Interface_Static::SetIVal("write.iges.brep.mode", 1);
  });
}

// Reports `offset + scale * position` of the indicator to the sink. (`Show` must not throw: OCCT
// calls it from progress-scope destructors.)
class RangeIndicator : public Message_ProgressIndicator {
public:
  RangeIndicator(const ProgressSink &sink, double offset, double scale)
      : sink_(sink), offset_(offset), scale_(scale) {}

  bool UserBreak() override { return progress_user_break(sink_); }

  void Show(const Message_ProgressScope &, const bool) override {
    progress_show(sink_, offset_ + scale_ * GetPosition());
  }

private:
  const ProgressSink &sink_;
  double offset_;
  double scale_;
};

// A file read in 64 KiB chunks that reports `scale * bytes read / size` and returns end of file
// once the sink asks to stop, so a parser reading it ends early.
class ProgressFileBuf : public std::streambuf {
public:
  ProgressFileBuf(const std::string &path, const ProgressSink &sink, double scale)
      : sink_(sink), scale_(scale), buffer_(1 << 16) {
    OSD_OpenStream(in_, path.c_str(), std::ios::in | std::ios::binary);
    if (!in_.is_open()) {
      fail("cannot open " + path);
    }
    in_.seekg(0, std::ios::end);
    const std::streamoff end = in_.tellg();
    in_.seekg(0, std::ios::beg);
    size_ = end > 0 ? static_cast<double>(end) : 1.0;
  }

  bool cancelled() const { return cancelled_; }

protected:
  int_type underflow() override {
    if (gptr() < egptr()) {
      return traits_type::to_int_type(*gptr());
    }
    if (cancelled_) {
      return traits_type::eof();
    }
    if (progress_user_break(sink_)) {
      cancelled_ = true;
      return traits_type::eof();
    }
    in_.read(buffer_.data(), static_cast<std::streamsize>(buffer_.size()));
    const std::streamsize got = in_.gcount();
    if (got <= 0) {
      return traits_type::eof();
    }
    done_ += static_cast<double>(got);
    progress_show(sink_, scale_ * std::min(1.0, done_ / size_));
    setg(buffer_.data(), buffer_.data(), buffer_.data() + got);
    return traits_type::to_int_type(*gptr());
  }

private:
  const ProgressSink &sink_;
  double scale_;
  std::ifstream in_;
  std::vector<char> buffer_;
  double size_ = 1.0;
  double done_ = 0.0;
  bool cancelled_ = false;
};

// A fresh XCAF document in mm with no application (none of the application's shared state).
occ::handle<TDocStd_Document> new_doc() {
  occ::handle<TDocStd_Document> doc = new TDocStd_Document("BinXCAF");
  XCAFDoc_DocumentTool::Set(doc->Main(), false);
  XCAFDoc_DocumentTool::SetLengthUnit(doc, 1.0, UnitsMethods_LengthUnit_Millimeter);
  return doc;
}

// Drops the document's attributes, which reference each other (otherwise it leaks).
void close_doc(const occ::handle<TDocStd_Document> &doc) {
  if (!doc.IsNull()) {
    doc->Main().Root().ForgetAllAttributes(true);
  }
}

std::string utf8(const TCollection_ExtendedString &text) {
  return TCollection_AsciiString(text).ToCString();
}

bool label_name(const TDF_Label &label, std::string &out) {
  occ::handle<TDataStd_Name> name;
  if (!label.FindAttribute(TDataStd_Name::GetID(), name)) {
    return false;
  }
  out = utf8(name->Get());
  return true;
}

void set_name(const TDF_Label &label, bool has, const rust::String &name) {
  if (has) {
    TDataStd_Name::Set(label, TCollection_ExtendedString(std::string(name).c_str(), true));
  } else {
    // XCAF names new shape labels after their type ("SOLID"); an unnamed node stays unnamed.
    label.ForgetAttribute(TDataStd_Name::GetID());
  }
}

bool label_color(const TDF_Label &label, std::array<float, 4> &out) {
  Quantity_ColorRGBA rgba;
  if (!XCAFDoc_ColorTool::GetColor(label, XCAFDoc_ColorSurf, rgba) &&
      !XCAFDoc_ColorTool::GetColor(label, XCAFDoc_ColorGen, rgba)) {
    return false;
  }
  // Quantity_Color keeps linear RGB.
  const Quantity_Color &rgb = rgba.GetRGB();
  out = {static_cast<float>(rgb.Red()), static_cast<float>(rgb.Green()),
         static_cast<float>(rgb.Blue()), rgba.Alpha()};
  return true;
}

Quantity_ColorRGBA rgba_of(const std::array<float, 4> &c) {
  return Quantity_ColorRGBA(Quantity_Color(c[0], c[1], c[2], Quantity_TOC_RGB), c[3]);
}

OcMat34 mat_of(const TopLoc_Location &loc) {
  const gp_Trsf t = loc.Transformation();
  OcMat34 m{};
  for (int r = 1; r <= 3; ++r) {
    for (int c = 1; c <= 4; ++c) {
      m.m[static_cast<std::size_t>((r - 1) * 4 + (c - 1))] = t.Value(r, c);
    }
  }
  return m;
}

TopLoc_Location loc_of(const OcMat34 &m) {
  gp_Trsf t;
  t.SetValues(m.m[0], m.m[1], m.m[2], m.m[3], m.m[4], m.m[5], m.m[6], m.m[7], m.m[8], m.m[9],
              m.m[10], m.m[11]);
  return TopLoc_Location(t);
}

bool is_identity(const OcMat34 &m) {
  static const double id[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
  for (std::size_t i = 0; i < 12; ++i) {
    if (m.m[i] != id[i]) {
      return false;
    }
  }
  return true;
}

// Gives every edge of `shape` that lacks a 3D curve one built from its pcurves
// (`ShapeFix_Edge::FixAddCurve3d`), in place. Only for shapes this reader made, before anything
// indexes or checks them: such edges (IGES can produce them) crash the index and BRepCheck
// (BREP-10 leftover). Returns the number of edges repaired; `failed` counts the ones it could not.
std::uint32_t repair_missing_curves3d(const TopoDS_Shape &shape, std::uint32_t &failed) {
  TopTools_IndexedMapOfShape edges;
  TopExp::MapShapes(shape, TopAbs_EDGE, edges);
  ShapeFix_Edge fixer;
  std::uint32_t repaired = 0;
  for (int i = 1; i <= edges.Extent(); ++i) {
    const TopoDS_Edge &edge = TopoDS::Edge(edges(i));
    if (BRep_Tool::Degenerated(edge)) {
      continue;
    }
    double first = 0.0;
    double last = 0.0;
    if (!BRep_Tool::Curve(edge, first, last).IsNull()) {
      continue;
    }
    fixer.FixAddCurve3d(edge);
    if (BRep_Tool::Curve(edge, first, last).IsNull()) {
      ++failed;
    } else {
      ++repaired;
    }
  }
  return repaired;
}

// Walks an XCAF document's free shapes into `out`.
class TreeReader {
public:
  TreeReader(const occ::handle<TDocStd_Document> &doc, bool names, bool colors, XdeData &out)
      : shapes_(XCAFDoc_DocumentTool::ShapeTool(doc->Main())), names_(names), colors_(colors),
        out_(out) {}

  void read() {
    TDF_LabelSequence roots;
    shapes_->GetFreeShapes(roots);
    for (int i = 1; i <= roots.Length(); ++i) {
      emit(roots(i), NONE, TopLoc_Location());
    }
  }

private:
  void emit(const TDF_Label &label, std::uint32_t parent, const TopLoc_Location &loc) {
    TDF_Label referred = label;
    if (XCAFDoc_ShapeTool::IsReference(label)) {
      XCAFDoc_ShapeTool::GetReferredShape(label, referred);
    }
    OcXdeNode node{};
    node.parent = parent;
    node.transform = mat_of(loc);
    node.proto = NONE;
    if (names_) {
      std::string name;
      if ((label != referred && label_name(label, name) && !name.empty()) ||
          label_name(referred, name)) {
        node.has_name = true;
        node.name = rust::String(name);
      }
    }
    if (colors_) {
      std::array<float, 4> color{};
      if ((label != referred && label_color(label, color)) || label_color(referred, color)) {
        node.has_color = true;
        std::copy(color.begin(), color.end(), node.color.begin());
      }
    }
    if (XCAFDoc_ShapeTool::IsAssembly(referred)) {
      const auto self = static_cast<std::uint32_t>(out_.nodes.size());
      out_.nodes.push_back(std::move(node));
      TDF_LabelSequence components;
      XCAFDoc_ShapeTool::GetComponents(referred, components, false);
      for (int i = 1; i <= components.Length(); ++i) {
        emit(components(i), self, XCAFDoc_ShapeTool::GetLocation(components(i)));
      }
      return;
    }
    node.proto = proto_of(referred);
    out_.nodes.push_back(std::move(node));
  }

  std::uint32_t proto_of(const TDF_Label &label) {
    TCollection_AsciiString entry;
    TDF_Tool::Entry(label, entry);
    const std::string key = entry.ToCString();
    const auto found = protos_.find(key);
    if (found != protos_.end()) {
      return found->second;
    }
    const auto index = static_cast<std::uint32_t>(out_.shapes.size());
    protos_.emplace(key, index);
    const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(label);
    OcXdeProto meta{};
    std::string name;
    if (names_ && label_name(label, name)) {
      meta.has_name = true;
      meta.name = rust::String(name);
    }
    std::array<float, 4> color{};
    if (colors_ && label_color(label, color)) {
      meta.has_color = true;
      std::copy(color.begin(), color.end(), meta.color.begin());
    }
    if (!shape.IsNull()) {
      std::uint32_t failed = 0;
      const std::uint32_t repaired = repair_missing_curves3d(shape, failed);
      if (repaired > 0) {
        out_.status.warnings.push_back(rust::String(
            "prototype " + std::to_string(index) + ": built the missing 3D curves of " +
            std::to_string(repaired) + " edge(s)"));
      }
      if (failed > 0) {
        out_.status.warnings.push_back(rust::String(
            "prototype " + std::to_string(index) + ": " + std::to_string(failed) +
            " edge(s) have no 3D curve and none could be built"));
      }
      if (colors_) {
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(shape, TopAbs_FACE, faces);
        for (int f = 1; f <= faces.Extent(); ++f) {
          TDF_Label sub;
          std::array<float, 4> face_color{};
          if (shapes_->FindSubShape(label, faces(f), sub) && label_color(sub, face_color)) {
            OcXdeFaceColor fc{};
            fc.proto = index;
            fc.face = static_cast<std::uint32_t>(f - 1);
            std::copy(face_color.begin(), face_color.end(), fc.color.begin());
            out_.face_colors.push_back(fc);
          }
        }
      }
    }
    out_.shapes.push_back(shape);
    out_.protos.push_back(std::move(meta));
    return index;
  }

  occ::handle<XCAFDoc_ShapeTool> shapes_;
  bool names_;
  bool colors_;
  XdeData &out_;
  std::map<std::string, std::uint32_t> protos_;
};

// A private copy of `shape` with its location baked into the geometry, so XCAF stores it as a
// prototype of its own (no reference split) and the caller's TShapes are never put into an
// XCAF compound.
TopoDS_Shape private_copy(const TopoDS_Shape &shape) {
  if (shape.Location().IsIdentity()) {
    BRepBuilderAPI_Copy copier(shape, true, false);
    return copier.Shape();
  }
  BRepBuilderAPI_Transform mover(shape.Located(TopLoc_Location()),
                                 shape.Location().Transformation(), true);
  return mover.Shape();
}

// Builds an XCAF document from plain nodes (see `xde_write_step`).
class TreeWriter {
public:
  TreeWriter(const OcShapeList &protos, rust::Slice<const OcXdeProto> meta,
             rust::Slice<const OcXdeFaceColor> face_colors, rust::Slice<const OcXdeNode> nodes)
      : doc_(new_doc()), shapes_(XCAFDoc_DocumentTool::ShapeTool(doc_->Main())),
        colors_(XCAFDoc_DocumentTool::ColorTool(doc_->Main())), protos_(protos), meta_(meta),
        face_colors_(face_colors), nodes_(nodes) {
    if (meta.size() != protos.shapes.size()) {
      fail("write: one prototype record per shape expected");
    }
    copies_.resize(protos.shapes.size());
    labels_.resize(protos.shapes.size());
    free_used_.assign(protos.shapes.size(), false);
    component_refs_.assign(protos.shapes.size(), 0);
    children_.resize(nodes.size());
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      const OcXdeNode &n = nodes[i];
      if (n.parent != NONE) {
        if (n.parent >= i) {
          fail("write: a node's parent must come before it");
        }
        children_[n.parent].push_back(static_cast<std::uint32_t>(i));
      } else {
        roots_.push_back(static_cast<std::uint32_t>(i));
      }
      if (n.proto != NONE && n.proto >= protos.shapes.size()) {
        fail("write: a node names a prototype that does not exist");
      }
    }
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (nodes[i].parent != NONE && nodes[i].proto != NONE && children_[i].empty()) {
        ++component_refs_[nodes[i].proto];
      }
    }
  }

  ~TreeWriter() { close_doc(doc_); }

  const occ::handle<TDocStd_Document> &build() {
    for (const std::uint32_t r : roots_) {
      const OcXdeNode &n = nodes_[r];
      if (is_part(r)) {
        TDF_Label label;
        if (component_refs_[n.proto] == 0 && !free_used_[n.proto] && is_identity(n.transform)) {
          free_used_[n.proto] = true;
          label = proto_label(n.proto);
        } else {
          // A part that is also an assembly component, or placed: a located copy of its own.
          label = new_part(n.proto, &n.transform);
        }
        if (n.has_name) {
          set_name(label, true, n.name);
        }
        if (n.has_color) {
          colors_->SetColor(label, rgba_of(n.color), XCAFDoc_ColorSurf);
        }
      } else {
        assembly(r, is_identity(n.transform) ? nullptr : &n.transform);
      }
    }
    shapes_->UpdateAssemblies();
    return doc_;
  }

private:
  bool is_part(std::uint32_t n) const {
    return nodes_[n].proto != NONE && children_[n].empty();
  }

  TDF_Label new_part(std::uint32_t p, const OcMat34 *placed) {
    TopoDS_Shape shape;
    if (placed != nullptr) {
      // A copy of its own with the placement baked into the geometry: XCAF would store a
      // located shape as an assembly referring to the unlocated one.
      shape = private_copy(protos_.shapes[p].Moved(loc_of(*placed)));
    } else {
      if (copies_[p].IsNull()) {
        copies_[p] = private_copy(protos_.shapes[p]);
      }
      shape = copies_[p];
    }
    const TDF_Label label = shapes_->AddShape(shape, false, false);
    const OcXdeProto &m = meta_[p];
    set_name(label, m.has_name, m.name);
    if (m.has_color) {
      colors_->SetColor(label, rgba_of(m.color), XCAFDoc_ColorSurf);
    }
    bool any = false;
    for (const OcXdeFaceColor &fc : face_colors_) {
      any = any || fc.proto == p;
    }
    if (any) {
      TopTools_IndexedMapOfShape faces;
      TopExp::MapShapes(shape, TopAbs_FACE, faces);
      for (const OcXdeFaceColor &fc : face_colors_) {
        if (fc.proto != p) {
          continue;
        }
        if (fc.face >= static_cast<std::uint32_t>(faces.Extent())) {
          fail("write: a face colour names a face the prototype does not have");
        }
        const TDF_Label sub = shapes_->AddSubShape(label, faces(static_cast<int>(fc.face) + 1));
        if (!sub.IsNull()) {
          colors_->SetColor(sub, rgba_of(fc.color), XCAFDoc_ColorSurf);
        }
      }
    }
    return label;
  }

  TDF_Label proto_label(std::uint32_t p) {
    if (labels_[p].IsNull()) {
      labels_[p] = new_part(p, nullptr);
    }
    return labels_[p];
  }

  // `n` as a new assembly label (sub-assemblies are not shared); `placed` folds a root's
  // placement into its components.
  TDF_Label assembly(std::uint32_t n, const OcMat34 *placed) {
    const OcXdeNode &node = nodes_[n];
    const TDF_Label label = shapes_->NewShape();
    set_name(label, node.has_name, node.name);
    if (node.has_color) {
      colors_->SetColor(label, rgba_of(node.color), XCAFDoc_ColorSurf);
    }
    for (const std::uint32_t c : children_[n]) {
      const OcXdeNode &child = nodes_[c];
      TopLoc_Location loc = loc_of(child.transform);
      if (placed != nullptr) {
        loc = loc_of(*placed) * loc;
      }
      TDF_Label component;
      if (is_part(c)) {
        const TDF_Label proto = proto_label(child.proto);
        component = shapes_->AddComponent(label, proto, loc);
        const OcXdeProto &m = meta_[child.proto];
        // The instance name (the NAUO's, which the reader gives back); an unnamed instance
        // takes its part's name, as the reader would show it anyway (else it reads back as the
        // NAUO's id).
        if (child.has_name) {
          set_name(component, true, child.name);
        } else {
          set_name(component, m.has_name, m.name);
        }
        const bool same_color = child.has_color == m.has_color &&
                                (!m.has_color || child.color == m.color);
        if (child.has_color && !same_color) {
          colors_->SetColor(component, rgba_of(child.color), XCAFDoc_ColorSurf);
        }
      } else {
        const TDF_Label sub = assembly(c, nullptr);
        component = shapes_->AddComponent(label, sub, loc);
        set_name(component, child.has_name, child.name);
      }
    }
    return label;
  }

  occ::handle<TDocStd_Document> doc_;
  occ::handle<XCAFDoc_ShapeTool> shapes_;
  occ::handle<XCAFDoc_ColorTool> colors_;
  const OcShapeList &protos_;
  rust::Slice<const OcXdeProto> meta_;
  rust::Slice<const OcXdeFaceColor> face_colors_;
  rust::Slice<const OcXdeNode> nodes_;
  std::vector<TopoDS_Shape> copies_;
  std::vector<TDF_Label> labels_;
  std::vector<bool> free_used_;
  std::vector<std::uint32_t> component_refs_;
  std::vector<std::vector<std::uint32_t>> children_;
  std::vector<std::uint32_t> roots_;
};

DESTEP_Parameters::WriteMode_StepSchema step_schema(std::uint8_t schema) {
  switch (schema) {
  case 0: return DESTEP_Parameters::WriteMode_StepSchema_AP203;
  case 1: return DESTEP_Parameters::WriteMode_StepSchema_AP214IS;
  case 2: return DESTEP_Parameters::WriteMode_StepSchema_AP242DIS;
  default: fail("write: unknown STEP schema " + std::to_string(schema));
  }
}

UnitsMethods_LengthUnit length_unit(double unit_mm) {
  struct Known {
    double mm;
    UnitsMethods_LengthUnit unit;
  };
  static const Known known[] = {
      {1.0, UnitsMethods_LengthUnit_Millimeter}, {25.4, UnitsMethods_LengthUnit_Inch},
      {304.8, UnitsMethods_LengthUnit_Foot},     {1000.0, UnitsMethods_LengthUnit_Meter},
      {10.0, UnitsMethods_LengthUnit_Centimeter}, {0.001, UnitsMethods_LengthUnit_Micron},
  };
  for (const Known &k : known) {
    if (std::abs(unit_mm - k.mm) <= 1e-12 * k.mm) {
      return k.unit;
    }
  }
  fail("write: unsupported length unit of " + std::to_string(unit_mm) + " mm");
}

const char *iges_unit(double unit_mm) {
  switch (length_unit(unit_mm)) {
  case UnitsMethods_LengthUnit_Inch: return "IN";
  case UnitsMethods_LengthUnit_Foot: return "FT";
  case UnitsMethods_LengthUnit_Meter: return "M";
  case UnitsMethods_LengthUnit_Centimeter: return "CM";
  case UnitsMethods_LengthUnit_Micron: return "UM";
  default: return "MM";
  }
}

// AP203 names a person and an organisation after the OS user and the host's IP address, and IGES
// puts the user name in its header: neither belongs in a file someone shares.
void anonymise(const occ::handle<StepData_StepModel> &model) {
  if (model.IsNull()) {
    return;
  }
  const occ::handle<TCollection_HAsciiString> unknown = new TCollection_HAsciiString("unknown");
  for (int i = 1; i <= model->NbEntities(); ++i) {
    const occ::handle<Standard_Transient> &entity = model->Value(i);
    const occ::handle<StepBasic_Person> person = occ::handle<StepBasic_Person>::DownCast(entity);
    if (!person.IsNull()) {
      person->SetId(unknown);
      person->UnSetFirstName();
      person->UnSetLastName();
    }
    const occ::handle<StepBasic_Organization> organization =
        occ::handle<StepBasic_Organization>::DownCast(entity);
    if (!organization.IsNull()) {
      organization->SetId(unknown);
    }
  }
}

void finish_read(const occ::handle<TDocStd_Document> &doc, bool names, bool colors,
                 XdeData &data) {
  TreeReader reader(doc, names, colors, data);
  reader.read();
}

} // namespace

std::unique_ptr<OcXde> xde_read_step(rust::Str path, bool names, bool colors,
                                     const ProgressSink &sink) {
  init_translators();
  auto out = std::make_unique<OcXde>();
  XdeData &data = *out->data;
  const std::string file(path);
  // Parsing is the first 20 % of the progress (about its share of the time), the transfer the
  // rest.
  ProgressFileBuf buffer(file, sink, 0.2);
  std::istream stream(&buffer);
  auto owned = std::make_shared<STEPCAFControl_Reader>();
  STEPCAFControl_Reader &reader = *owned;
  // A cancelled read hands the reader and its parsed model to the caller to free elsewhere.
  const auto cancelled = [&]() -> std::unique_ptr<OcXde> {
    data.status.cancelled = true;
    data.keep = owned;
    return std::move(out);
  };
  reader.SetColorMode(colors);
  reader.SetNameMode(names);
  reader.SetLayerMode(false);
  reader.SetPropsMode(false);
  reader.SetGDTMode(false);
  reader.SetMatMode(false);
  reader.SetViewMode(false);
  reader.SetMetaMode(false);
  DESTEP_Parameters params;
  params.ReadColor = colors;
  params.ReadName = names;
  params.ReadLayer = false;
  params.ReadProps = false;
  params.ReadMetadata = false;
  const IFSelect_ReturnStatus read = reader.ChangeReader().ReadStream(file.c_str(), params, stream);
  if (buffer.cancelled()) {
    return cancelled();
  }
  if (read != IFSelect_RetDone) {
    data.status.errors.push_back(rust::String("cannot parse " + file + " as STEP"));
    return out;
  }
  NCollection_Sequence<TCollection_AsciiString> lengths;
  NCollection_Sequence<TCollection_AsciiString> angles;
  NCollection_Sequence<TCollection_AsciiString> solid_angles;
  reader.ChangeReader().FileUnits(lengths, angles, solid_angles);
  for (int i = 1; i <= lengths.Length(); ++i) {
    data.status.unit_names.push_back(rust::String(lengths(i).ToCString()));
  }
  occ::handle<TDocStd_Document> doc = new_doc();
  occ::handle<RangeIndicator> indicator = new RangeIndicator(sink, 0.2, 0.8);
  bool transferred = false;
  try {
    Message_ProgressScope scope(indicator->Start(), "amg read STEP", 1);
    transferred = reader.Transfer(doc, scope.Next());
  } catch (...) {
    close_doc(doc);
    throw;
  }
  if (progress_user_break(sink)) {
    close_doc(doc);
    return cancelled();
  }
  if (!transferred) {
    data.status.errors.push_back(rust::String("cannot transfer " + file + " from STEP"));
    close_doc(doc);
    return out;
  }
  try {
    finish_read(doc, names, colors, data);
  } catch (...) {
    close_doc(doc);
    throw;
  }
  close_doc(doc);
  return out;
}

std::unique_ptr<OcXde> xde_read_iges(rust::Str path, bool names, bool colors,
                                     const ProgressSink &sink) {
  init_translators();
  auto out = std::make_unique<OcXde>();
  XdeData &data = *out->data;
  const std::string file(path);
  if (progress_user_break(sink)) {
    data.status.cancelled = true;
    return out;
  }
  IGESCAFControl_Reader reader;
  reader.SetColorMode(colors);
  reader.SetNameMode(names);
  reader.SetLayerMode(false);
  if (reader.ReadFile(file.c_str()) != IFSelect_RetDone) {
    data.status.errors.push_back(rust::String("cannot parse " + file + " as IGES"));
    return out;
  }
  progress_show(sink, 0.4);
  const occ::handle<IGESData_IGESModel> model =
      occ::handle<IGESData_IGESModel>::DownCast(reader.Model());
  if (!model.IsNull()) {
    data.status.iges_unit_flag = model->GlobalSection().UnitFlag();
  }
  occ::handle<TDocStd_Document> doc = new_doc();
  occ::handle<RangeIndicator> indicator = new RangeIndicator(sink, 0.4, 0.6);
  bool transferred = false;
  try {
    Message_ProgressScope scope(indicator->Start(), "amg read IGES", 1);
    transferred = reader.Transfer(doc, scope.Next());
  } catch (...) {
    close_doc(doc);
    throw;
  }
  if (progress_user_break(sink)) {
    data.status.cancelled = true;
    close_doc(doc);
    return out;
  }
  if (!transferred) {
    data.status.errors.push_back(rust::String("cannot transfer " + file + " from IGES"));
    close_doc(doc);
    return out;
  }
  try {
    finish_read(doc, names, colors, data);
  } catch (...) {
    close_doc(doc);
    throw;
  }
  close_doc(doc);
  return out;
}

OcXdeStatus xde_status(const OcXde &xde) { return xde.data->status; }

rust::Vec<OcXdeNode> xde_nodes(const OcXde &xde) {
  rust::Vec<OcXdeNode> out;
  for (const OcXdeNode &n : xde.data->nodes) {
    out.push_back(n);
  }
  return out;
}

rust::Vec<OcXdeProto> xde_protos(const OcXde &xde) {
  rust::Vec<OcXdeProto> out;
  for (const OcXdeProto &p : xde.data->protos) {
    out.push_back(p);
  }
  return out;
}

rust::Vec<OcXdeFaceColor> xde_face_colors(const OcXde &xde) {
  rust::Vec<OcXdeFaceColor> out;
  for (const OcXdeFaceColor &c : xde.data->face_colors) {
    out.push_back(c);
  }
  return out;
}

std::unique_ptr<OcShape> xde_proto_shape(const OcXde &xde, std::uint32_t i) {
  if (i >= xde.data->shapes.size()) {
    fail("xde_proto_shape: no prototype " + std::to_string(i));
  }
  return std::make_unique<OcShape>(xde.data->shapes[i]);
}

OcXdeStatus xde_write_step(rust::Str path, const OcXdeWriteParams &write,
                           const OcShapeList &protos, rust::Slice<const OcXdeProto> meta,
                           rust::Slice<const OcXdeFaceColor> face_colors,
                           rust::Slice<const OcXdeNode> nodes, const ProgressSink &sink) {
  init_translators();
  OcXdeStatus status{};
  DESTEP_Parameters params;
  params.WriteSchema = step_schema(write.schema);
  params.WriteUnit = length_unit(write.unit_mm);
  params.WriteColor = true;
  params.WriteName = true;
  params.WriteLayer = false;
  params.WriteProps = false;
  params.WriteMaterial = false;
  TreeWriter tree(protos, meta, face_colors, nodes);
  const occ::handle<TDocStd_Document> &doc = tree.build();
  STEPCAFControl_Writer writer;
  writer.SetColorMode(true);
  writer.SetNameMode(true);
  writer.SetLayerMode(false);
  writer.SetPropsMode(false);
  occ::handle<RangeIndicator> indicator = new RangeIndicator(sink, 0.0, 0.9);
  bool transferred = false;
  {
    Message_ProgressScope scope(indicator->Start(), "amg write STEP", 1);
    transferred = writer.Transfer(doc, params, STEPControl_AsIs, nullptr, scope.Next());
  }
  if (progress_user_break(sink)) {
    status.cancelled = true;
    return status;
  }
  if (!transferred) {
    status.errors.push_back(rust::String("STEP transfer failed"));
    return status;
  }
  anonymise(writer.ChangeWriter().Model());
  const std::string file(path);
  if (writer.Write(file.c_str()) != IFSelect_RetDone) {
    status.errors.push_back(rust::String("cannot write " + file));
    return status;
  }
  progress_show(sink, 1.0);
  return status;
}

OcXdeStatus xde_write_iges(rust::Str path, const OcXdeWriteParams &write,
                           const OcShapeList &protos,
                           rust::Slice<const OcXdeProto> meta,
                           rust::Slice<const OcXdeFaceColor> face_colors,
                           rust::Slice<const OcXdeNode> nodes, const ProgressSink &sink) {
  init_translators();
  OcXdeStatus status{};
  const char *unit = iges_unit(write.unit_mm);
  TreeWriter tree(protos, meta, face_colors, nodes);
  const occ::handle<TDocStd_Document> &doc = tree.build();
  // The default constructor takes the unit and the B-rep mode from the IGES statics (the
  // `(session, unit)` one forces faces mode); the caller holds the global lock.
  Interface_Static::SetCVal("write.iges.unit", unit);
  Interface_Static::SetIVal("write.iges.brep.mode", 1);
  IGESCAFControl_Writer writer;
  writer.SetColorMode(true);
  writer.SetNameMode(true);
  writer.SetLayerMode(false);
  occ::handle<RangeIndicator> indicator = new RangeIndicator(sink, 0.0, 0.9);
  bool transferred = false;
  {
    Message_ProgressScope scope(indicator->Start(), "amg write IGES", 1);
    transferred = writer.Transfer(doc, scope.Next());
  }
  if (progress_user_break(sink)) {
    status.cancelled = true;
    return status;
  }
  if (!transferred) {
    status.errors.push_back(rust::String("IGES transfer failed"));
    return status;
  }
  {
    IGESData_GlobalSection header = writer.Model()->GlobalSection();
    header.SetAuthorName(new TCollection_HAsciiString("unknown"));
    writer.Model()->SetGlobalSection(header);
  }
  const std::string file(path);
  if (!writer.Write(file.c_str())) {
    status.errors.push_back(rust::String("cannot write " + file));
    return status;
  }
  progress_show(sink, 1.0);
  return status;
}

std::unique_ptr<OcShape> xde_guard_probe(const OcShape &shape, std::uint32_t edge) {
  BRepBuilderAPI_Copy copier(shape.get(), true, false);
  const TopoDS_Shape copy = copier.Shape();
  TopTools_IndexedMapOfShape edges;
  TopExp::MapShapes(copy, TopAbs_EDGE, edges);
  if (edge >= static_cast<std::uint32_t>(edges.Extent())) {
    fail("xde_guard_probe: no edge " + std::to_string(edge));
  }
  const TopoDS_Edge &stripped = TopoDS::Edge(edges(static_cast<int>(edge) + 1));
  BRep_Builder builder;
  builder.UpdateEdge(stripped, occ::handle<Geom_Curve>(), TopLoc_Location(),
                     BRep_Tool::Tolerance(stripped));
  double first = 0.0;
  double last = 0.0;
  if (!BRep_Tool::Curve(stripped, first, last).IsNull()) {
    fail("xde_guard_probe: the 3D curve was not removed");
  }
  std::uint32_t failed = 0;
  const std::uint32_t repaired = repair_missing_curves3d(copy, failed);
  if (repaired != 1 || failed != 0) {
    fail("xde_guard_probe: repaired " + std::to_string(repaired) + ", failed " +
         std::to_string(failed));
  }
  return std::make_unique<OcShape>(copy);
}

} // namespace amg::occt
