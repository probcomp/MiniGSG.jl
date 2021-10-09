SceneGraph() = MG.MetaDiGraph()

"""
Returns the root nodes (their LightGraphs integer identifiers, not their
`:name` property) of the connected components (trees) of `g`, which is assumed
to be a directed forest.
"""
function rootsOfForest(g::LG.AbstractGraph)
    @assert LG.is_directed(g)
    setdiff(LG.vertices(g), Set(LG.dst(e) for e in LG.edges(g)))
end

"""
"Compiles" a scene graph, returning an mapping of vertex name to absolute pose.

The graph `g` is expected to be a directed forest having the following
properties:

* vertex property `:name`, present for all vertices
* vertex property `:shape`, of type [`Shape`](@ref), present for all vertices
* vertex property `:absolutePose`, of type [`Pose`](@ref), present for the root
  vertex of each connected component of `g`
* edge property `:contact`, of type [`ShapeContact`](@ref), present for all
  edges (as currently, all edges represent contact relations).

Compilation consists of a graph traversal applying the relative
poses implied by the contact relation.
"""
function floatingPosesOf(g::MG.AbstractMetaGraph)::OrderedDict
    # Strategy: create a stripped-down copy of `g` that has only `:name`,
    # `:absolutePose` and `:contact` properties.  Call the mutating `toFloating!`
    # on this copy, and extract the needed absolute poses from there.
    g_ = MG.MetaDiGraph(LG.SimpleDiGraph(g))
    for i in LG.vertices(g_)
        MG.set_indexing_prop!(g_, i, :name, MG.get_prop(g, i, :name))
        MG.set_prop!(g_, i, :shape, MG.get_prop(g, i, :shape))
        if MG.has_prop(g, i, :absolutePose)
            MG.set_prop!(g_, i, :absolutePose, MG.get_prop(g, i, :absolutePose))
        end
    end
    for (srcName, dstName) in namedEdges(g)
        setContact!(g_, srcName, dstName, getContact(g, srcName, dstName))
    end
    toFloating!(g_)
    return OrderedDict(vertName => getAbsolutePose(g_, vertName)
                       for vertName in namedVertices(g_))
end


"""
Mutating version of [`floatingPosesOf`](@ref), which converts `g` to a floating
scene graph by assigning an `:absolutePose` property to all vertices that don't
already have one and then deleting all the edges.
"""
function toFloating!(g::MG.AbstractMetaGraph)
    @assert LG.is_directed(g)
    forestScan!(g; edgeField=:contact,
                vertexField=:absolutePose,
                vertexMetaFields=(:shape,),
                agg=contactGraphAgg,
                # Disallow overwrites: if there was an overwrite, then the pose
                # of an object was overspecified (both an absolute pose and
                # contact relation were given).
                allowOverwrite=false)
    for e in collect(LG.edges(g))
        LG.rem_edge!(g, e)
    end
    g
end


"""
Like the higher-order function `scan`, but operates on a forest rather than a
list.

Given a directed forest (represented as a `MetaDiGraph`) that has:

- Metadata attached to each edge
- A value attached to the root vertex of each connected component (via a
  vertex-level property with key `vertexField`)
- Optionally, also metadata attached to each vertex

and given a function `agg`, `forestScan!` traverses the graph and sets all
vertex values (i.e. a vertex-level property with key `vertexField`) according
to the rule

```julia
destVertexValue = agg(sourceVertexValue, edgeMeta,
                      sourceVertexMeta, destVertexMeta)
```

Here `sourceVertexMeta` and `destVertexMeta` are `NamedTuple`s, whose keys are
specified by the `vertexMetaFields` argument of `forestScan!`, and whose value
at each key is the vertex-level property (see
[`MetaGraphs.get_prop`](https://juliagraphs.github.io/MetaGraphs.jl/latest/metagraphs.html#MetaGraphs.get_prop-Tuple{MetaGraphs.AbstractMetaGraph,Symbol}))
with that key.  For keys that do not occur as properties of the source vertex
(resp., of destination vertex), the corresponding value in `sourceVertexMeta`
(resp., `destVertexMeta`) will be set to `nothing`.

Optionally, the caller may restrict the traversal to a subset of the connected
components, by setting `roots` to the set of roots of those connected
components.  By default, all components will be traversed.

If the boolean flag `allowOverwrite` is `false` (its default value), this
function enforces that the property `vertexField` is not already present at the
vertices where this function would set it (i.e., at non-root vertices).
"""
function forestScan!(g::MG.AbstractMetaGraph;
                     edgeField, vertexField, agg, roots=nothing,
                     vertexMetaFields::Tuple{Vararg{Symbol}}=(),
                     allowOverwrite=false)
    @assert LG.is_directed(g)
    if roots == nothing
        roots = rootsOfForest(g)
    end
    for root in roots
        t = LG.bfs_tree(g, root)
        remainingVerts = [root]
        while !isempty(remainingVerts)
            s = pop!(remainingVerts)
            for n in LG.neighbors(t, s)
                @assert allowOverwrite || !MG.has_prop(g, n, vertexField)
                MG.set_prop!(g, n, vertexField,
                             agg(MG.get_prop(g, s, vertexField),
                                 MG.get_prop(g, LG.Edge(s, n), edgeField),
                                 NamedTuple{vertexMetaFields}(Tuple(
                                         get(MG.props(g, s), prop, nothing)
                                         for prop in vertexMetaFields)),
                                 NamedTuple{vertexMetaFields}(Tuple(
                                         get(MG.props(g, n), prop, nothing)
                                         for prop in vertexMetaFields))))
            end
            append!(remainingVerts, LG.neighbors(t, s))
        end
    end
end

"""$(raw"""
Implements ``\mathrm{Agg}`` (see [Agg and ForestScan](@ref)) for contact graphs.

Given the 3D `Shape`s of the parent and child objects, and the
`ShapeContactRelation` describing how they are in contact, and the absolute
pose of the parent object, this function outputs the absolute pose of the child
object.
""")"""
function contactGraphAgg(absolutePose::Pose,
                         contact::ShapeContact,
                         parent::NamedTuple{(:shape,), Tuple{S}}
                             where {S<:Shape},
                         child::NamedTuple{(:shape,), Tuple{S}}
                             where {S<:Shape},
                         )::Pose
      absolutePose * getRelativePoseFromContact(
                         parent.shape, child.shape, contact)
end


"""
Returns the 6DOF absolute pose of the object with name `name` in the given
scene graph.

Performance note: If you need the absolute pose of all objects in the scene, it
is more efficient to call [`floatingPosesOf`](@ref) or [`toFloating!`](@ref)
rather than calling `getAbsolutePose` on every vertex.
"""
function getAbsolutePose(g::MG.AbstractMetaGraph, name)::Pose
    @assert LG.is_directed(g)
    path = _pathFromRootTo(g, g[name, :name])
    pose = MG.get_prop(g, path[1], :absolutePose)
    for (parent, child) in consecutivePairs(path)
        contact = get_prop(g, LG.Edge(parent, child), :contact)
        pose = contactGraphAgg(pose, contact,
                               (shape=MG.get_prop(g, parent, :shape),),
                               (shape=MG.get_prop(g, child, :shape),))
    end
    return pose
end


"""
Computes the relative pose of `child`'s coordinate frame with respect to
`parent`'s coordinate frame, given that `child` and `parent` are in contact as
described by `contact`.  Note that the aforementioned coordinate frames are the
coordinate frames of the whole shape, not of individual contact planes (see
`getContactPlane`).
"""
function getRelativePoseFromContact(parent::Shape,
                                    child::Shape,
                                    contact::ShapeContact
                                   )::Pose
    parentPlaneFrame = getContactPlane(parent, contact.parentFamilyId,
                                       contact.parentPlaneParams...)
    childPlaneFrame = getContactPlane(child, contact.childFamilyId,
                                      contact.childPlaneParams...)
    (parentPlaneFrame
     * planarContactTo6DOF(contact.planarContact)
     / childPlaneFrame)
end


"""
Returns the path to vertex `v`, starting from the root vertex of the connected
component of vertex `v`, in the graph `g` which is assumed to be a directed
forest.  Here vertices are represented by LightGraphs integer IDs.
"""
function _pathFromRootTo(g::LG.AbstractGraph, v::Integer)
    @assert LG.is_directed(g)
    revPath = []
    while true
        push!(revPath, v)
        parents = LG.inneighbors(g, v)
        if isempty(parents)
            break
        end
        (v,) = parents
    end
    path = reverse(revPath)
    return path
end
