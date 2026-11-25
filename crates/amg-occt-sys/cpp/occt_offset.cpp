// Shell (thick solid), offset and thicken (S02 §5, BREP-07). Declarations and categories are in
// occt_shim.hxx.
//
// All three run BRepOffset_MakeOffset (the algorithm behind BRepOffsetAPI_MakeThickSolid and
// BRepOffsetAPI_MakeOffsetShape, used directly for per-face offsets, thickening and the
// progress range) on a private BRepBuilderAPI_Copy of the input (category C): the offset
// algorithm updates tolerances and pcurves of the faces it trims. The history is reported from
// the input through the copy, in amg-brep's index of the caller's shape.
//
// Before the build, every face that is offset is sampled for its principal curvatures: an offset
// `d` (along the solid's outward normal) towards a centre of curvature closer than |d| collapses
// the offset surface (scale factor 1 - d k <= 0), and such faces are the op's offending faces.
// OCCT itself does not refuse these offsets; it builds self-intersecting or empty results.

#include "amg-occt-sys/cpp/occt_history.hxx"
#include "amg-occt-sys/cpp/occt_shim.hxx"

#include "amg-occt-sys/src/bridge.rs.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <BRepAdaptor_Surface.hxx>
#include <BRep_Builder.hxx>
#include <BRep_CurveOn2Surfaces.hxx>
#include <BRep_CurveRepresentation.hxx>
#include <BRep_TEdge.hxx>
#include <BRep_Tool.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepGProp.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRepOffset_Error.hxx>
#include <BRepOffset_MakeOffset.hxx>
#include <BRepOffset_Mode.hxx>
#include <BRepTools.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_JoinType.hxx>
#include <Message_ProgressIndicator.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <Message_ProgressScope.hxx>
#include <Standard_ConstructionError.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>

namespace amg::occt {

namespace {

using namespace history;

// OcOffsetParams::mode.
constexpr std::uint8_t MODE_SHELL = 0;
constexpr std::uint8_t MODE_OFFSET = 1;
constexpr std::uint8_t MODE_THICKEN = 2;

// Role codes of OcOp::roles beyond the blend ones (amg-brep `ops::history_n`).
constexpr std::uint8_t SHELL_INNER = 6;
constexpr std::uint8_t OFFSET_FACE = 7;

// An error that amg-brep reports as `InvalidInput` (same prefix as occt_blend.cpp).
constexpr const char *INVALID = "invalid: ";

// Curvature samples per parameter direction of each offset face.
constexpr int SAMPLES = 9;

// A face whose scale factor 1 - d k falls below this somewhere collapses.
constexpr double MIN_SCALE = 1e-7;

[[noreturn]] void fail(const std::string &message) {
  throw Standard_ConstructionError(message.c_str());
}

class OffsetProgress : public Message_ProgressIndicator {
public:
  explicit OffsetProgress(const ProgressSink &sink) : sink_(sink) {}

  bool UserBreak() override { return progress_user_break(sink_); }

  void Show(const Message_ProgressScope &, const bool) override {
    progress_show(sink_, GetPosition());
  }

private:
  const ProgressSink &sink_;
};

const char *offset_error(BRepOffset_Error error) {
  switch (error) {
  case BRepOffset_NoError:
    return "BRepOffset_NoError";
  case BRepOffset_BadNormalsOnGeometry:
    return "BRepOffset_BadNormalsOnGeometry";
  case BRepOffset_C0Geometry:
    return "BRepOffset_C0Geometry";
  case BRepOffset_NullOffset:
    return "BRepOffset_NullOffset";
  case BRepOffset_NotConnectedShell:
    return "BRepOffset_NotConnectedShell";
  case BRepOffset_CannotTrimEdges:
    return "BRepOffset_CannotTrimEdges";
  case BRepOffset_CannotFuseVertices:
    return "BRepOffset_CannotFuseVertices";
  case BRepOffset_CannotExtentEdge:
    return "BRepOffset_CannotExtentEdge";
  case BRepOffset_UserBreak:
    return "BRepOffset_UserBreak";
  case BRepOffset_MixedConnectivity:
    return "BRepOffset_MixedConnectivity";
  default:
    return "BRepOffset_UnknownError";
  }
}

void push_offending_face(OcOp &op, std::uint32_t face) {
  for (std::size_t j = 0; j < op.offending_indices.size(); ++j) {
    if (op.offending_kinds[j] == 0 && op.offending_indices[j] == face) {
      return;
    }
  }
  op.offending_operands.push_back(0);
  op.offending_kinds.push_back(0);
  op.offending_indices.push_back(face);
}

// The input's faces and their copies, in the input's index order.
struct Copied {
  BRepBuilderAPI_Copy copier;
  TopoDS_Shape base;
  ShapeMap faces; // copy of input face i at index i + 1

  Copied(const TopoDS_Shape &input, const OcIndex &index) : copier(input, false, false) {
    base = copier.Shape();
    for (int i = 1; i <= index.faces.Extent(); ++i) {
      faces.Add(image(index.faces(i)));
    }
  }

  // The copy of `s`, in the orientation of `s` (Modified gives the clone of its TShape in an
  // orientation of its own; the clone has the same geometric sense).
  TopoDS_Shape image(const TopoDS_Shape &s) {
    const NCollection_List<TopoDS_Shape> &images = copier.Modified(s);
    if (images.IsEmpty()) {
      fail("offset: an input sub-shape has no copy");
    }
    TopoDS_Shape out = images.First();
    out.Orientation(s.Orientation());
    return out;
  }

  const TopoDS_Face &face(std::uint32_t f) const {
    if (f >= static_cast<std::uint32_t>(faces.Extent())) {
      fail("offset: face index out of range");
    }
    return TopoDS::Face(faces(static_cast<int>(f) + 1));
  }
};

// The smallest scale factor 1 - d k over a grid of `face` (offset `d` along its outward
// normal), and the radius of curvature where it is reached (infinite on flat faces).
struct Collapse {
  double scale = 1.0;
  double radius = INFINITY;
};

Collapse collapse_of(const TopoDS_Face &face, double d) {
  Collapse out;
  double u0 = 0.0;
  double u1 = 0.0;
  double v0 = 0.0;
  double v1 = 0.0;
  BRepTools::UVBounds(face, u0, u1, v0, v1);
  const BRepAdaptor_Surface surface(face, false);
  // The curvature signs of BRepLProp follow the surface normal D1u x D1v; the solid's outward
  // normal is its opposite on a reversed face.
  const double dn = face.Orientation() == TopAbs_REVERSED ? -d : d;
  for (int i = 0; i < SAMPLES; ++i) {
    for (int j = 0; j < SAMPLES; ++j) {
      // Cell centres: the poles of a sphere (no normal) are never sampled.
      const double u = u0 + (u1 - u0) * (i + 0.5) / SAMPLES;
      const double v = v0 + (v1 - v0) * (j + 0.5) / SAMPLES;
      BRepLProp_SLProps props(surface, u, v, 2, 1e-9);
      if (!props.IsNormalDefined() || !props.IsCurvatureDefined()) {
        continue;
      }
      // k > 0 where the surface bends towards its normal; 1 - d k scales the offset surface.
      for (double k : {props.MaxCurvature(), props.MinCurvature()}) {
        const double scale = 1.0 - dn * k;
        if (scale < out.scale) {
          out.scale = scale;
          out.radius = 1.0 / std::abs(k);
        }
      }
    }
  }
  return out;
}

std::string number(double v) {
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.6g", v);
  return buf;
}

// Every offset face whose offset collapses it becomes offending. Returns whether any did.
bool check_curvature(OcOp &op, const Copied &c, const std::vector<double> &offsets) {
  bool any = false;
  for (std::size_t f = 0; f < offsets.size(); ++f) {
    const double d = offsets[f];
    if (d == 0.0) {
      continue;
    }
    const Collapse k = collapse_of(c.face(static_cast<std::uint32_t>(f)), d);
    if (k.scale < MIN_SCALE) {
      push_offending_face(op, static_cast<std::uint32_t>(f));
      op.errors.push_back("face " + std::to_string(f) + ": offset " + number(d) +
                          " mm reaches its centre of curvature (radius " + number(k.radius) +
                          " mm)");
      any = true;
    }
  }
  return any;
}

int solid_count(const TopoDS_Shape &s) {
  ShapeMap solids;
  TopExp::MapShapes(s, TopAbs_SOLID, solids);
  return solids.Extent();
}

// The sort key of a face or a compound child: area (largest first), then the centroid.
struct Key {
  double area;
  double x;
  double y;
  double z;
  int index; // the original position, last resort for congruent coincident faces

  bool operator<(const Key &o) const {
    if (area != o.area) {
      return area > o.area;
    }
    if (x != o.x) {
      return x < o.x;
    }
    if (y != o.y) {
      return y < o.y;
    }
    if (z != o.z) {
      return z < o.z;
    }
    return index < o.index;
  }
};

Key key_of(const TopoDS_Shape &s, int index) {
  GProp_GProps props;
  BRepGProp::SurfaceProperties(s, props);
  const gp_Pnt c = props.CentreOfMass();
  return Key{props.Mass(), c.X(), c.Y(), c.Z(), index};
}

// `s` with the faces of every shell (and the children of every compound) in key order.
// BRepOffset_MakeOffset collects faces in maps hashed by TShape address, so its face order, and
// with it the sub-shape indices and the BinTools bytes, change from run to run (S02 §7:
// canonicalise). The faces themselves are kept: only shells, solids and compounds are rebuilt.
TopoDS_Shape canonical(const TopoDS_Shape &s) {
  BRep_Builder builder;
  const TopAbs_ShapeEnum type = s.ShapeType();
  if (type == TopAbs_SHELL || type == TopAbs_COMPOUND) {
    std::vector<std::pair<Key, TopoDS_Shape>> children;
    int i = 0;
    for (TopoDS_Iterator it(s); it.More(); it.Next(), ++i) {
      const TopoDS_Shape child = type == TopAbs_SHELL ? it.Value() : canonical(it.Value());
      children.emplace_back(key_of(child, i), child);
    }
    std::sort(children.begin(), children.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });
    TopoDS_Shape out;
    if (type == TopAbs_SHELL) {
      TopoDS_Shell shell;
      builder.MakeShell(shell);
      out = shell;
    } else {
      TopoDS_Compound compound;
      builder.MakeCompound(compound);
      out = compound;
    }
    for (const auto &child : children) {
      builder.Add(out, child.second);
    }
    out.Closed(s.Closed());
    return out;
  }
  if (type == TopAbs_SOLID) {
    TopoDS_Solid solid;
    builder.MakeSolid(solid);
    for (TopoDS_Iterator it(s); it.More(); it.Next()) {
      builder.Add(solid, canonical(it.Value()));
    }
    return solid;
  }
  return s;
}

// Rewrites the regularity records of every edge of `s` (BRep_CurveOn2Surfaces: the continuity
// between two of its faces, from BRepOffset's EncodeRegularity; a seam's own record on its closed
// surface is left alone) in face index order: OCCT stores each face pair in the
// order it meets it in an address-hashed map, which the BinTools bytes keep. The edges of `s` are
// private to this op (copies or new), so they are changed in place.
void canonical_regularity(const TopoDS_Shape &s) {
  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>
      edge_faces;
  TopExp::MapShapesAndUniqueAncestors(s, TopAbs_EDGE, TopAbs_FACE, edge_faces);
  ShapeMap faces;
  TopExp::MapShapes(s, TopAbs_FACE, faces);
  BRep_Builder builder;
  for (int i = 1; i <= edge_faces.Extent(); ++i) {
    const TopoDS_Edge &edge = TopoDS::Edge(edge_faces.FindKey(i));
    std::vector<TopoDS_Face> around;
    for (const TopoDS_Shape &f : edge_faces(i)) {
      around.push_back(TopoDS::Face(f));
    }
    std::sort(around.begin(), around.end(), [&](const TopoDS_Face &a, const TopoDS_Face &b) {
      return faces.FindIndex(a) < faces.FindIndex(b);
    });
    struct Regularity {
      TopoDS_Face a;
      TopoDS_Face b;
      GeomAbs_Shape continuity;
    };
    std::vector<Regularity> keep;
    for (std::size_t x = 0; x < around.size(); ++x) {
      for (std::size_t y = x + 1; y < around.size(); ++y) {
        if (BRep_Tool::HasContinuity(edge, around[x], around[y])) {
          keep.push_back({around[x], around[y], BRep_Tool::Continuity(edge, around[x], around[y])});
        }
      }
    }
    if (keep.empty()) {
      continue;
    }
    occ::handle<BRep_TEdge> tedge = occ::down_cast<BRep_TEdge>(edge.TShape());
    if (tedge.IsNull()) {
      continue;
    }
    NCollection_List<occ::handle<BRep_CurveRepresentation>> &curves = tedge->ChangeCurves();
    NCollection_List<occ::handle<BRep_CurveRepresentation>>::Iterator it(curves);
    while (it.More()) {
      if (it.Value()->IsKind(STANDARD_TYPE(BRep_CurveOn2Surfaces))) {
        curves.Remove(it);
      } else {
        it.Next();
      }
    }
    for (const Regularity &r : keep) {
      builder.Continuity(edge, r.a, r.b, r.continuity);
    }
  }
}

// A hollow solid: `outer` and `inner` (outward-oriented closed shells, `inner` inside `outer`)
// with the material between them.
TopoDS_Shape hollow(const TopoDS_Shape &outer, const TopoDS_Shape &inner) {
  BRep_Builder builder;
  TopoDS_Solid solid;
  builder.MakeSolid(solid);
  builder.Add(solid, outer);
  builder.Add(solid, inner.Reversed());
  return solid;
}

// The one shell of `s`; null when it has none or several.
TopoDS_Shape only_shell(const TopoDS_Shape &s) {
  ShapeMap shells;
  TopExp::MapShapes(s, TopAbs_SHELL, shells);
  return shells.Extent() == 1 ? shells(1) : TopoDS_Shape();
}

// The history through the copy and the offset, and the roles of the new faces.
//
// OCCT's `Modified` and `Generated` of a copied sub-shape (offset faces of faces, arc faces of
// edges and vertices, offset edges) are its images. An offset moves the input, so images of the
// input's kind are `Modified` and the others `Generated`; a shell or thicken keeps the input (as
// its copy, `Modified` 1:1) and all images are `Generated`. New faces that are no image of
// anything (the side walls of a thickened face) are `Generated` from the input edges on them.
void record(OcOp &op, BRepOffset_MakeOffset &maker, Copied &c, const TopoDS_Shape &input,
            std::uint8_t mode, const std::vector<bool> &removed) {
  const Maps out(op.shape);
  const Maps in(input);
  const auto in_result = [&](const TopoDS_Shape &s) {
    const int k = kind_of(s);
    return k >= 0 && out.m[k].Contains(s);
  };
  const auto images = [&](const TopoDS_Shape &copy) {
    Shapes all = shapes_of(maker.Modified(copy));
    for (const TopoDS_Shape &g : shapes_of(maker.Generated(copy))) {
      all.push_back(g);
    }
    Shapes kept;
    for (const TopoDS_Shape &s : all) {
      if (!s.IsSame(copy) && in_result(s)) {
        kept.push_back(s);
      }
    }
    return kept;
  };

  ShapeMap copies; // result faces that are copies of input faces
  for (int f = 1; f <= in.m[0].Extent(); ++f) {
    const TopoDS_Shape copy = c.image(in.m[0](f));
    if (in_result(copy)) {
      copies.Add(copy);
    }
  }
  ShapeMap on_removed; // copied edges and vertices of removed faces
  for (std::size_t f = 0; f < removed.size(); ++f) {
    if (removed[f]) {
      const TopoDS_Face &face = c.face(static_cast<std::uint32_t>(f));
      TopExp::MapShapes(face, TopAbs_EDGE, on_removed);
      TopExp::MapShapes(face, TopAbs_VERTEX, on_removed);
    }
  }
  // The offset skin: images of faces that are offset, and of edges and vertices not on a
  // removed face (arcs). Images of removed faces (the rim at the opening) are not skin.
  ShapeMap skin;
  ShapeMap derived;
  for (int kind = 0; kind < 3; ++kind) {
    for (int i = 1; i <= in.m[kind].Extent(); ++i) {
      const TopoDS_Shape copy = c.image(in.m[kind](i));
      const bool off_skin =
          kind == 0 ? removed[static_cast<std::size_t>(i - 1)] : on_removed.Contains(copy);
      for (const TopoDS_Shape &image : images(copy)) {
        if (kind_of(image) != 0 || copies.Contains(image)) {
          continue;
        }
        derived.Add(image);
        if (!off_skin) {
          skin.Add(image);
        }
      }
    }
  }
  std::vector<ShapeMap> walls; // new faces no image explains: the face, then its edges
  for (int i = 1; i <= out.m[0].Extent(); ++i) {
    const TopoDS_Shape &face = out.m[0](i);
    if (!copies.Contains(face) && !derived.Contains(face)) {
      ShapeMap edges;
      edges.Add(face);
      TopExp::MapShapes(face, TopAbs_EDGE, edges);
      walls.push_back(std::move(edges));
    }
  }
  const auto walls_on = [&](const TopoDS_Shape &copy) {
    Shapes on;
    if (copy.ShapeType() == TopAbs_EDGE) {
      for (const ShapeMap &w : walls) {
        if (w.Contains(copy)) {
          on.push_back(w(1));
        }
      }
    }
    return on;
  };

  Source source;
  if (mode == MODE_OFFSET) {
    source.modified = [&](const TopoDS_Shape &s) -> Shapes {
      const TopoDS_Shape copy = c.image(s);
      if (in_result(copy)) {
        return {copy};
      }
      Shapes same;
      for (const TopoDS_Shape &image : images(copy)) {
        if (image.ShapeType() == copy.ShapeType()) {
          same.push_back(image);
        }
      }
      return same;
    };
    source.generated = [&](const TopoDS_Shape &s) -> Shapes {
      const TopoDS_Shape copy = c.image(s);
      Shapes other;
      for (const TopoDS_Shape &image : images(copy)) {
        if (image.ShapeType() != copy.ShapeType()) {
          other.push_back(image);
        }
      }
      return other;
    };
  } else {
    source.modified = [&](const TopoDS_Shape &s) -> Shapes {
      const TopoDS_Shape copy = c.image(s);
      if (in_result(copy)) {
        return {copy};
      }
      return {};
    };
    source.generated = [&](const TopoDS_Shape &s) -> Shapes {
      const TopoDS_Shape copy = c.image(s);
      Shapes all = images(copy);
      for (const TopoDS_Shape &w : walls_on(copy)) {
        all.push_back(w);
      }
      return all;
    };
  }
  record_operands(op, {input}, source, {}, false);

  // Roles: the offset skin is `ShellInner` / `OffsetFace`, every other new face (the rim at a
  // removed face, the side walls of a thickened face) `Lateral`.
  const std::uint8_t skin_role = mode == MODE_SHELL ? SHELL_INNER : OFFSET_FACE;
  for (int i = 1; i <= out.m[0].Extent(); ++i) {
    const TopoDS_Shape &face = out.m[0](i);
    if (copies.Contains(face)) {
      continue;
    }
    op.role_faces.push_back(static_cast<std::uint32_t>(i - 1));
    op.roles.push_back(skin.Contains(face) ? skin_role : LATERAL);
    op.role_args.push_back(0);
  }
}

} // namespace

std::unique_ptr<OcOp> offset_op(const OcShape &shape, const OcIndex &index,
                                const OcOffsetParams &params,
                                rust::Slice<const std::uint32_t> remove,
                                rust::Slice<const std::uint32_t> faces,
                                rust::Slice<const double> offsets, const ProgressSink &sink) {
  const TopoDS_Shape &input = shape.get();
  if (input.IsNull()) {
    fail("offset: the shape is null");
  }
  if (params.mode > MODE_THICKEN) {
    fail("offset: unknown mode");
  }
  if (faces.size() != offsets.size()) {
    fail("offset: per-face lists differ in length");
  }
  auto op = std::make_unique<OcOp>();
  Copied c(input, index);
  const auto n = static_cast<std::size_t>(c.faces.Extent());
  std::vector<double> per_face(n, params.offset);
  std::vector<bool> removed(n, false);
  for (std::uint32_t f : remove) {
    c.face(f); // range check
    removed[f] = true;
    per_face[f] = 0.0;
  }
  for (std::size_t j = 0; j < faces.size(); ++j) {
    c.face(faces[j]);
    if (removed[faces[j]]) {
      op->errors.push_back(std::string(INVALID) + "face " + std::to_string(faces[j]) +
                           " is removed and has a thickness");
      return op;
    }
    per_face[faces[j]] = offsets[j];
  }
  if (check_curvature(*op, c, per_face)) {
    return op;
  }

  const GeomAbs_JoinType join = params.join == 1 ? GeomAbs_Intersection : GeomAbs_Arc;
  BRepOffset_MakeOffset maker;
  maker.Initialize(c.base, params.offset, params.tol, BRepOffset_Skin, false, false, join,
                   params.mode == MODE_THICKEN, false);
  for (std::uint32_t f : remove) {
    maker.AddFace(c.face(f));
  }
  for (std::size_t j = 0; j < faces.size(); ++j) {
    maker.SetOffsetOnFace(c.face(faces[j]), offsets[j]);
  }
  // MakeThickSolid without closing faces gives the offset skin alone: a shell with no removed
  // face is the offset solid and the input assembled into one hollow solid.
  const bool hollow_shell = params.mode == MODE_SHELL && remove.empty();
  TopoDS_Shape input_shell;
  if (hollow_shell) {
    input_shell = only_shell(c.base);
    if (input_shell.IsNull()) {
      op->errors.push_back(std::string(INVALID) +
                           "a shell without removed faces needs a solid with one shell");
      return op;
    }
  }
  occ::handle<OffsetProgress> indicator = new OffsetProgress(sink);
  try {
    Message_ProgressScope scope(indicator->Start(), "amg offset", 1);
    if (params.mode == MODE_SHELL && !hollow_shell) {
      maker.MakeThickSolid(scope.Next());
    } else {
      maker.MakeOffsetShape(scope.Next());
    }
  } catch (const Standard_Failure &failure) {
    op->errors.push_back(format_failure(failure));
  }
  if (progress_user_break(sink) || maker.Error() == BRepOffset_UserBreak) {
    op->cancelled = true;
    return op;
  }
  if (op->errors.empty() && !maker.IsDone()) {
    op->errors.emplace_back(offset_error(maker.Error()));
  }
  TopoDS_Shape result;
  if (op->errors.empty()) {
    result = maker.Shape();
    if (hollow_shell && !result.IsNull()) {
      const TopoDS_Shape offset_shell = only_shell(result);
      if (offset_shell.IsNull()) {
        op->errors.emplace_back("the offset of the solid is not one shell");
        op->shape = result;
      } else if (params.offset < 0.0) {
        result = hollow(input_shell, offset_shell);
      } else {
        result = hollow(offset_shell, input_shell);
      }
    }
  }
  if (op->errors.empty()) {
    const bool needs_solid = params.mode != MODE_OFFSET || solid_count(c.base) > 0;
    if (result.IsNull()) {
      op->errors.emplace_back("the result is empty");
    } else if (needs_solid && solid_count(result) == 0) {
      op->errors.emplace_back("the result has no solid");
      op->shape = result;
    }
  }
  if (!op->errors.empty()) {
    // Faces OCCT's input check blames (bad normals, C0 geometry).
    try {
      const TopoDS_Shape &bad = maker.GetBadShape();
      if (!bad.IsNull()) {
        ShapeMap bad_faces;
        TopExp::MapShapes(bad, TopAbs_FACE, bad_faces);
        for (int i = 1; i <= bad_faces.Extent(); ++i) {
          const int f = c.faces.FindIndex(bad_faces(i));
          if (f != 0) {
            push_offending_face(*op, static_cast<std::uint32_t>(f - 1));
          }
        }
      }
    } catch (const Standard_Failure &) {
    }
    return op;
  }
  op->shape = canonical(result);
  canonical_regularity(op->shape);
  record(*op, maker, c, input, params.mode, removed);
  return op;
}

} // namespace amg::occt
