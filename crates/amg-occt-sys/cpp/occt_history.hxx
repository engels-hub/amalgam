// History extraction shared by every shape op (S02 §6, BREP-05).
//
// An op reports, for every face, edge and vertex of every operand (TopExp::MapShapes order,
// amg-brep's index), what it became in the op's final result, in index terms of that result:
// `Kept` (the same sub-shape is in the result), `Modified` / `Generated` images, else `Deleted`.
// Roles label result faces. The relations come from one OCCT algorithm (a `Source`: a
// BRepBuilderAPI_MakeShape, a BRepTools_History, or op-specific accessors such as a sweep's
// FirstShape/LastShape) followed by the op's internal steps (copy, draft, UnifySameDomain, ...),
// each a `Step` that maps a shape of the previous stage to its images in the next. So callers
// see one input → output step whatever the op does inside.
//
// The Rust side (amg-brep `ops::history_n`) turns the flat lists into `ShapeHistory` and applies
// the split order and the completeness rule.

#pragma once

#include "amg-occt-sys/cpp/occt_shim.hxx"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include <BRepBuilderAPI_MakeShape.hxx>
#include <BRepTools_History.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopoDS_Shape.hxx>

namespace amg::occt::history {

// Relation codes of OcOp::relations (amg-brep `ops::history_n`).
constexpr std::uint8_t MODIFIED = 0;
constexpr std::uint8_t GENERATED = 1;
constexpr std::uint8_t DELETED = 2;
constexpr std::uint8_t KEPT = 3;

// Role codes of OcOp::roles (amg-brep `ops::history_n`).
constexpr std::uint8_t START_CAP = 0;
constexpr std::uint8_t END_CAP = 1;
constexpr std::uint8_t LATERAL = 2;

using Shapes = std::vector<TopoDS_Shape>;
// One internal step of an op: where a shape of the previous stage went.
using Step = std::function<Shapes(const TopoDS_Shape &)>;

inline Shapes shapes_of(const NCollection_List<TopoDS_Shape> &list) {
  Shapes out;
  for (const TopoDS_Shape &s : list) {
    out.push_back(s);
  }
  return out;
}

// 0 face, 1 edge, 2 vertex; -1 for every other kind.
inline int kind_of(const TopoDS_Shape &s) {
  switch (s.ShapeType()) {
  case TopAbs_FACE:
    return 0;
  case TopAbs_EDGE:
    return 1;
  case TopAbs_VERTEX:
    return 2;
  default:
    return -1;
  }
}

// The face, edge and vertex maps of a shape, in amg-brep's index order.
struct Maps {
  ShapeMap m[3];
  explicit Maps(const TopoDS_Shape &s) {
    TopExp::MapShapes(s, TopAbs_FACE, m[0]);
    TopExp::MapShapes(s, TopAbs_EDGE, m[1]);
    TopExp::MapShapes(s, TopAbs_VERTEX, m[2]);
  }
};

// Shapes after the later `steps` of an op, in order.
inline Shapes through(const std::vector<Step> &steps, Shapes shapes) {
  for (const Step &step : steps) {
    Shapes next;
    for (const TopoDS_Shape &s : shapes) {
      for (TopoDS_Shape &image : step(s)) {
        next.push_back(std::move(image));
      }
    }
    shapes = std::move(next);
  }
  return shapes;
}

// The step of a BRepTools_History (UnifySameDomain, ShapeFix, ...): a removed shape has no
// image, a modified one its images, any other shape passes unchanged. `history` must outlive the
// step.
inline Step history_step(const occ::handle<BRepTools_History> &history) {
  return [history](const TopoDS_Shape &s) -> Shapes {
    if (history.IsNull()) {
      return {s};
    }
    if (history->IsRemoved(s)) {
      return {};
    }
    const NCollection_List<TopoDS_Shape> &modified = history->Modified(s);
    if (!modified.IsEmpty()) {
      return shapes_of(modified);
    }
    return {s};
  };
}

// The step of a BRepBuilderAPI_MakeShape that modifies shapes (copy, draft, transform): its
// `Modified` images, else the shape itself. `builder` must outlive the step.
inline Step modified_step(BRepBuilderAPI_MakeShape &builder) {
  BRepBuilderAPI_MakeShape *b = &builder;
  return [b](const TopoDS_Shape &s) -> Shapes {
    const NCollection_List<TopoDS_Shape> &modified = b->Modified(s);
    if (!modified.IsEmpty()) {
      return shapes_of(modified);
    }
    return {s};
  };
}

// Where an operand sub-shape went in the op's main algorithm (before the internal steps).
struct Source {
  std::function<Shapes(const TopoDS_Shape &)> modified;
  std::function<Shapes(const TopoDS_Shape &)> generated;
};

// A BRepBuilderAPI_MakeShape's (booleans, transforms, fillets, ...) `Modified` and `Generated`.
// `builder` must outlive the source.
inline Source source_of(BRepBuilderAPI_MakeShape &builder) {
  BRepBuilderAPI_MakeShape *b = &builder;
  return Source{[b](const TopoDS_Shape &s) { return shapes_of(b->Modified(s)); },
                [b](const TopoDS_Shape &s) { return shapes_of(b->Generated(s)); }};
}

// A BRepTools_History's `Modified` and `Generated` (an unmodified, unremoved shape is its own
// image).
inline Source source_of(const occ::handle<BRepTools_History> &history) {
  const Step modified = history_step(history);
  return Source{modified, [history](const TopoDS_Shape &s) -> Shapes {
                  if (history.IsNull()) {
                    return {};
                  }
                  return shapes_of(history->Generated(s));
                }};
}

// Collects an op history in index terms of the final result `op.shape`.
class Recorder {
public:
  // `multi_operand`: fill OcOp::in_operands (booleans); one-operand ops leave it empty.
  // `by_vertices`: match an image missing from the result by its vertices (see below).
  Recorder(OcOp &op, const Maps &out, bool multi_operand, bool by_vertices = false)
      : op_(op), out_(out), multi_(multi_operand), by_vertices_(by_vertices),
        has_role_(static_cast<std::size_t>(out.m[0].Extent()), false) {}

  // Records one relation; returns whether any output is in the result.
  bool relation(std::uint16_t operand, int kind, int index, std::uint8_t relation,
                const Shapes &outputs) {
    std::vector<std::pair<std::uint8_t, std::uint32_t>> found;
    for (const TopoDS_Shape &o : outputs) {
      const int k = kind_of(o);
      if (k < 0) {
        continue;
      }
      const int i = find(o, k);
      if (i == 0) {
        continue;
      }
      const std::pair<std::uint8_t, std::uint32_t> key{static_cast<std::uint8_t>(k),
                                                        static_cast<std::uint32_t>(i - 1)};
      if (std::find(found.begin(), found.end(), key) == found.end()) {
        found.push_back(key);
      }
    }
    if (found.empty()) {
      return false;
    }
    push(operand, kind, index, relation);
    for (const auto &[k, i] : found) {
      op_.out_kinds.push_back(k);
      op_.out_indices.push_back(i);
    }
    close();
    return true;
  }

  void kept(std::uint16_t operand, int kind, int index, int out_index) {
    push(operand, kind, index, KEPT);
    op_.out_kinds.push_back(static_cast<std::uint8_t>(kind));
    op_.out_indices.push_back(static_cast<std::uint32_t>(out_index));
    close();
  }

  void deleted(std::uint16_t operand, int kind, int index) {
    push(operand, kind, index, DELETED);
    close();
  }

  // Labels the result faces among `faces` that have no role yet.
  void role(const Shapes &faces, std::uint8_t role) {
    for (const TopoDS_Shape &f : faces) {
      if (kind_of(f) != 0) {
        continue;
      }
      const int i = find(f, 0);
      if (i == 0 || has_role_[static_cast<std::size_t>(i - 1)]) {
        continue;
      }
      has_role_[static_cast<std::size_t>(i - 1)] = true;
      op_.role_faces.push_back(static_cast<std::uint32_t>(i - 1));
      op_.roles.push_back(role);
    }
  }

private:
  // 1-based index of `s` (kind `k`) in the result; 0 when it is not there.
  int find(const TopoDS_Shape &s, int k) const {
    const int i = out_.m[k].FindIndex(s);
    if (i != 0 || !by_vertices_) {
      return i;
    }
    return by_vertex_match(s, k);
  }

  // The sorted vertex indices of `s` in the result, and its edge count; empty when a vertex is
  // not in the result.
  std::pair<std::vector<int>, int> signature(const TopoDS_Shape &s) const {
    ShapeMap vertices;
    TopExp::MapShapes(s, TopAbs_VERTEX, vertices);
    ShapeMap edges;
    TopExp::MapShapes(s, TopAbs_EDGE, edges);
    std::vector<int> out;
    for (int v = 1; v <= vertices.Extent(); ++v) {
      const int i = out_.m[2].FindIndex(vertices(v));
      if (i == 0) {
        return {};
      }
      out.push_back(i);
    }
    std::sort(out.begin(), out.end());
    return {out, edges.Extent()};
  }

  // BRepPrimAPI_MakeRevol replaces some swept faces and edges after the sweep (surfaces of
  // revolution canonised into planes, cones, ...) and `Generated` still returns the sweep's
  // originals. The vertices survive, so an image missing from the result is matched to the
  // one result face or edge (1-based; 0 when none or ambiguous) with the same vertices and
  // edge count.
  int by_vertex_match(const TopoDS_Shape &image, int kind) const {
    if (kind > 1) {
      return 0;
    }
    const auto want = signature(image);
    if (want.first.empty()) {
      return 0;
    }
    int found = 0;
    for (int i = 1; i <= out_.m[kind].Extent(); ++i) {
      if (signature(out_.m[kind](i)) == want) {
        if (found != 0) {
          return 0;
        }
        found = i;
      }
    }
    return found;
  }

  void push(std::uint16_t operand, int kind, int index, std::uint8_t relation) {
    if (multi_) {
      op_.in_operands.push_back(operand);
    }
    op_.in_kinds.push_back(static_cast<std::uint8_t>(kind));
    op_.in_indices.push_back(static_cast<std::uint32_t>(index));
    op_.relations.push_back(relation);
  }

  void close() { op_.out_starts.push_back(static_cast<std::uint32_t>(op_.out_kinds.size())); }

  OcOp &op_;
  const Maps &out_;
  bool multi_;
  bool by_vertices_;
  std::vector<bool> has_role_;
};

// The history of every face, edge and vertex of every operand through `source` and the later
// `steps`, in index terms of `op.shape`: `Kept` when the same sub-shape is in the result, else
// its `Modified` and `Generated` images (in that order) that reach the result, else `Deleted`.
// `operands` are the shapes the algorithm ran on, with the same sub-shape order as the callers'
// shapes (the caller's own handles, or `detached` / copied ones whose maps match).
inline void record_operands(OcOp &op, const std::vector<TopoDS_Shape> &operands,
                            const Source &source, const std::vector<Step> &steps,
                            bool multi_operand) {
  const Maps out(op.shape);
  Recorder rec(op, out, multi_operand);
  for (std::size_t o = 0; o < operands.size(); ++o) {
    const auto operand = static_cast<std::uint16_t>(o);
    const Maps in(operands[o]);
    for (int kind = 0; kind < 3; ++kind) {
      for (int i = 1; i <= in.m[kind].Extent(); ++i) {
        const TopoDS_Shape &s = in.m[kind](i);
        const int same = out.m[kind].FindIndex(s);
        if (same != 0) {
          rec.kept(operand, kind, i - 1, same - 1);
          continue;
        }
        const bool modified =
            rec.relation(operand, kind, i - 1, MODIFIED, through(steps, source.modified(s)));
        const bool generated =
            rec.relation(operand, kind, i - 1, GENERATED, through(steps, source.generated(s)));
        if (!modified && !generated) {
          rec.deleted(operand, kind, i - 1);
        }
      }
    }
  }
}

} // namespace amg::occt::history
