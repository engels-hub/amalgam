// Booleans, split and section (S02 §5, BREP-04). Declarations and categories are in
// occt_shim.hxx.
//
// Every operation is category N: BRepAlgoAPI with SetNonDestructive(true), so OCCT copies an
// argument sub-shape before it would enlarge its tolerance or add a pcurve, and the caller's
// TShapes are only read. Progress and cancellation go through the BREP-12 ProgressSink.
//
// The history is reported per operand (objects first, then tools) in index terms of the result:
// an input sub-shape that is in the result as is is `Kept`, otherwise `Modified` / `Generated`
// from BRepAlgoAPI_BuilderAlgo's history, else `Deleted`. Failures carry the BOPAlgo alert names
// and the operand sub-shapes the alerts name.

#include "amg-occt-sys/cpp/occt_history.hxx"
#include "amg-occt-sys/cpp/occt_profile.hxx"
#include "amg-occt-sys/cpp/occt_shim.hxx"

#include "amg-occt-sys/src/bridge.rs.h"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <BOPAlgo_Alerts.hxx>
#include <BOPAlgo_GlueEnum.hxx>
#include <BOPAlgo_Operation.hxx>
#include <BRepAlgoAPI_BooleanOperation.hxx>
#include <BRepAlgoAPI_BuilderAlgo.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BOPTools_AlgoTools.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Message_Alert.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <Message_Report.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_AlertWithShape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopLoc_Location.hxx>

namespace amg::occt {

namespace {

using namespace history;

// Same bridge as SinkProgress in occt_shim.cpp (kept file-local there).
class BoolProgress : public Message_ProgressIndicator {
public:
  explicit BoolProgress(const ProgressSink &sink) : sink_(sink) {}

  bool UserBreak() override { return progress_user_break(sink_); }

  void Show(const Message_ProgressScope &scope, const bool) override {
    profile::scope(scope);
    progress_show(sink_, GetPosition());
  }

private:
  const ProgressSink &sink_;
};

// The operand sub-shapes (faces, edges, vertices) an alert names, at most `limit` in all.
void offending_of(const TopoDS_Shape &shape, const std::vector<Maps> &inputs, OcOp &op,
                  std::size_t limit) {
  if (shape.IsNull() || op.offending_kinds.size() >= limit) {
    return;
  }
  const int k = kind_of(shape);
  if (k < 0) {
    for (TopoDS_Iterator it(shape); it.More(); it.Next()) {
      offending_of(it.Value(), inputs, op, limit);
    }
    return;
  }
  for (std::size_t o = 0; o < inputs.size(); ++o) {
    const int i = inputs[o].m[k].FindIndex(shape);
    if (i != 0) {
      for (std::size_t j = 0; j < op.offending_kinds.size(); ++j) {
        if (op.offending_operands[j] == o && op.offending_kinds[j] == k &&
            op.offending_indices[j] == static_cast<std::uint32_t>(i - 1)) {
          return;
        }
      }
      op.offending_operands.push_back(static_cast<std::uint16_t>(o));
      op.offending_kinds.push_back(static_cast<std::uint8_t>(k));
      op.offending_indices.push_back(static_cast<std::uint32_t>(i - 1));
      return;
    }
  }
}

// Alert names (deduplicated, in report order) and the shapes they name.
void collect_alerts(const BRepAlgoAPI_BuilderAlgo &algo, Message_Gravity gravity,
                    std::vector<std::string> &names, const std::vector<Maps> *inputs, OcOp *op) {
  const occ::handle<Message_Report> &report = algo.GetReport();
  if (report.IsNull()) {
    return;
  }
  for (const occ::handle<Message_Alert> &alert : report->GetAlerts(gravity)) {
    if (alert.IsNull()) {
      continue;
    }
    const std::string name = alert->GetMessageKey();
    if (std::find(names.begin(), names.end(), name) == names.end()) {
      names.push_back(name);
    }
    if (inputs != nullptr && op != nullptr) {
      occ::handle<TopoDS_AlertWithShape> with_shape =
          occ::down_cast<TopoDS_AlertWithShape>(alert);
      if (!with_shape.IsNull()) {
        offending_of(with_shape->GetShape(), *inputs, *op, 64);
      }
    }
  }
}

// After Build: cancellation, errors (with the shapes they name) and warnings into `op`.
// Returns whether the algorithm produced a usable result.
bool finish(OcOp &op, const BRepAlgoAPI_BuilderAlgo &algo, const std::vector<TopoDS_Shape> &operands,
            const ProgressSink &sink) {
  if (algo.HasError(STANDARD_TYPE(BOPAlgo_AlertUserBreak)) || progress_user_break(sink)) {
    op.cancelled = true;
    return false;
  }
  std::vector<Maps> inputs;
  inputs.reserve(operands.size());
  for (const TopoDS_Shape &s : operands) {
    inputs.emplace_back(s);
  }
  collect_alerts(algo, Message_Warning, op.warnings, &inputs, nullptr);
  if (algo.HasErrors() || !algo.IsDone()) {
    collect_alerts(algo, Message_Fail, op.errors, &inputs, &op);
    if (op.errors.empty()) {
      op.errors.emplace_back("NotDone");
    }
    return false;
  }
  return true;
}

void apply(BRepAlgoAPI_BuilderAlgo &algo, const OcBoolOpts &opts) {
  algo.SetNonDestructive(true);
  algo.SetFuzzyValue(opts.fuzzy);
  switch (opts.glue) {
  case 1:
    algo.SetGlue(BOPAlgo_GlueShift);
    break;
  case 2:
    algo.SetGlue(BOPAlgo_GlueFull);
    break;
  default:
    algo.SetGlue(BOPAlgo_GlueOff);
    break;
  }
  algo.SetCheckInverted(opts.check_inverted);
  algo.SetUseOBB(opts.use_obb);
  algo.SetRunParallel(opts.parallel);
  algo.SetToFillHistory(true);
}

// A private top-level TShape for `s` holding the same children (same location, orientation and
// flags, so the same TopExp::MapShapes order below the top). BOPAlgo puts an argument that the
// operation leaves untouched into the result compound as is, and TopoDS_Builder::Add then clears
// that TShape's Free flag: a write to the caller's shape that BinTools sees, non-destructive mode
// or not. Detaching makes that write land on the private copy. The children are already inside
// `s`, so their Free flag is already clear (Add only rewrites the same value there).
TopoDS_Shape detached(const TopoDS_Shape &s) {
  TopoDS_Shape copy = s.EmptyCopied();
  copy.Location(TopLoc_Location());
  copy.Orientation(TopAbs_FORWARD);
  BRep_Builder builder;
  for (TopoDS_Iterator it(s, false, false); it.More(); it.Next()) {
    builder.Add(copy, it.Value());
  }
  copy.Closed(s.Closed());
  copy.Orientable(s.Orientable());
  copy.Infinite(s.Infinite());
  copy.Convex(s.Convex());
  copy.Location(s.Location());
  copy.Orientation(s.Orientation());
  return copy;
}

std::vector<TopoDS_Shape> detached_all(const std::vector<TopoDS_Shape> &shapes) {
  std::vector<TopoDS_Shape> out;
  out.reserve(shapes.size());
  for (const TopoDS_Shape &s : shapes) {
    out.push_back(detached(s));
  }
  return out;
}

NCollection_List<TopoDS_Shape> list_of(const std::vector<TopoDS_Shape> &shapes) {
  NCollection_List<TopoDS_Shape> out;
  for (const TopoDS_Shape &s : shapes) {
    out.Append(s);
  }
  return out;
}

} // namespace

std::unique_ptr<OcShapeList> new_shape_list() { return std::make_unique<OcShapeList>(); }

void shape_list_push(OcShapeList &list, const OcShape &shape) {
  if (shape.get().IsNull()) {
    throw std::invalid_argument("shape_list_push: the shape is null");
  }
  list.shapes.push_back(shape.get());
}

std::unique_ptr<OcOp> boolean_op(std::uint8_t op_kind, const OcShapeList &objects,
                                 const OcShapeList &tools, const OcBoolOpts &opts,
                                 const ProgressSink &sink) {
  if (op_kind > 3) {
    throw std::invalid_argument("boolean_op: unknown operation");
  }
  if (objects.shapes.empty()) {
    throw std::invalid_argument("boolean_op: no objects");
  }
  if (op_kind != 3 && tools.shapes.empty()) {
    throw std::invalid_argument("boolean_op: no tools");
  }
  const profile::Phases phases("boolean");
  // Operands in history order (objects, then tools), each detached (see `detached`). Their
  // face/edge/vertex index order is the callers' shapes' order.
  const std::vector<TopoDS_Shape> args = detached_all(objects.shapes);
  const std::vector<TopoDS_Shape> tool_args = detached_all(tools.shapes);
  std::vector<TopoDS_Shape> operands = args;
  operands.insert(operands.end(), tool_args.begin(), tool_args.end());

  std::unique_ptr<BRepAlgoAPI_BuilderAlgo> algo;
  if (op_kind == 3) {
    auto splitter = std::make_unique<BRepAlgoAPI_Splitter>();
    splitter->SetArguments(list_of(args));
    splitter->SetTools(list_of(tool_args));
    algo = std::move(splitter);
  } else {
    auto boolean = std::make_unique<BRepAlgoAPI_BooleanOperation>();
    static constexpr BOPAlgo_Operation OPS[3] = {BOPAlgo_FUSE, BOPAlgo_CUT, BOPAlgo_COMMON};
    boolean->SetOperation(OPS[op_kind]);
    boolean->SetArguments(list_of(args));
    boolean->SetTools(list_of(tool_args));
    algo = std::move(boolean);
  }
  apply(*algo, opts);

  auto op = std::make_unique<OcOp>();
  occ::handle<BoolProgress> indicator = new BoolProgress(sink);
  {
    Message_ProgressScope scope(indicator->Start(), "amg boolean", 1);
    algo->Build(scope.Next());
  }
  phases.mark("built");
  if (!finish(*op, *algo, operands, sink)) {
    return op;
  }
  op->shape = algo->Shape();
  phases.mark("finished");
  record_operands(*op, operands, source_of(*algo), {}, true);
  phases.mark("history recorded");
  return op;
}

bool shape_has_inverted_solid(const OcShape &shape) {
  for (TopExp_Explorer it(shape.get(), TopAbs_SOLID); it.More(); it.Next()) {
    if (BOPTools_AlgoTools::IsInvertedSolid(TopoDS::Solid(it.Current()))) {
      return true;
    }
  }
  return false;
}

std::unique_ptr<OcOp> section_op(const OcShape &a, const OcShape &b, bool parallel,
                                 const ProgressSink &sink) {
  if (a.get().IsNull() || b.get().IsNull()) {
    throw std::invalid_argument("section_op: a shape is null");
  }
  const std::vector<TopoDS_Shape> operands{detached(a.get()), detached(b.get())};
  BRepAlgoAPI_Section section(operands[0], operands[1], false);
  section.Approximation(true);
  section.ComputePCurveOn1(false);
  section.ComputePCurveOn2(false);
  section.SetNonDestructive(true);
  section.SetRunParallel(parallel);
  section.SetToFillHistory(true);

  auto op = std::make_unique<OcOp>();
  occ::handle<BoolProgress> indicator = new BoolProgress(sink);
  {
    Message_ProgressScope scope(indicator->Start(), "amg section", 1);
    section.Build(scope.Next());
  }
  if (!finish(*op, section, operands, sink)) {
    return op;
  }
  op->shape = section.Shape();

  // Faces generate the section edges that lie on them (HasAncestorFaceOn1/2); vertices and
  // edges of the operands that are in the result are kept.
  const Maps out(op->shape);
  Recorder rec(*op, out, true);
  for (std::size_t o = 0; o < operands.size(); ++o) {
    const auto operand = static_cast<std::uint16_t>(o);
    const Maps in(operands[o]);
    std::vector<NCollection_List<TopoDS_Shape>> generated(
        static_cast<std::size_t>(in.m[0].Extent()));
    for (int e = 1; e <= out.m[1].Extent(); ++e) {
      const TopoDS_Edge &edge = TopoDS::Edge(out.m[1](e));
      TopoDS_Shape face;
      const bool found =
          o == 0 ? section.HasAncestorFaceOn1(edge, face) : section.HasAncestorFaceOn2(edge, face);
      if (!found) {
        continue;
      }
      const int f = in.m[0].FindIndex(face);
      if (f != 0) {
        generated[static_cast<std::size_t>(f - 1)].Append(edge);
      }
    }
    for (int kind = 0; kind < 3; ++kind) {
      for (int i = 1; i <= in.m[kind].Extent(); ++i) {
        if (kind > 0) {
          const int same = out.m[kind].FindIndex(in.m[kind](i));
          if (same != 0) {
            rec.kept(operand, kind, i - 1, same - 1);
          }
          continue;
        }
        rec.relation(operand, kind, i - 1, GENERATED,
                     shapes_of(generated[static_cast<std::size_t>(i - 1)]));
      }
    }
  }
  return op;
}

double volume_adaptive(const OcShape &shape, double eps) {
  GProp_GProps props;
  BRepGProp::VolumeProperties(shape.get(), props, eps);
  return props.Mass();
}

OcBoolStatus op_bool_status(const OcOp &op) {
  OcBoolStatus out;
  out.cancelled = op.cancelled;
  for (const std::string &e : op.errors) {
    out.errors.push_back(rust::String(e));
  }
  for (const std::string &w : op.warnings) {
    out.warnings.push_back(rust::String(w));
  }
  for (std::uint16_t o : op.offending_operands) {
    out.offending_operands.push_back(o);
  }
  for (std::uint8_t k : op.offending_kinds) {
    out.offending_kinds.push_back(k);
  }
  for (std::uint32_t i : op.offending_indices) {
    out.offending_indices.push_back(i);
  }
  return out;
}

} // namespace amg::occt
