_getName(g::LG.AbstractGraph, node) = MG.get_prop(g, node, :name)

namedEdges(g::LG.AbstractGraph) = [(_getName(g, LG.src(e)), _getName(g, LG.dst(e)))
                                   for e in LG.edges(g)]

vertexNames(g::LG.AbstractGraph) = [_getName(g, v) for v in LG.vertices(g)]
namedVertices = vertexNames

function addObject!(g, name, shape::Shape)
    LG.add_vertex!(g)
    v = length(LG.vertices(g))
    MG.set_indexing_prop!(g, v, :name, name)
    MG.set_prop!(g, v, :shape, shape)
end

function setPose!(g, name, pose::Pose)
    LG.indegree(g, g[name, :name]) == 0 || error(
        "this object's pose is already determined by a contact relation")
    MG.set_prop!(g, g[name, :name], :absolutePose, pose)
end

set_prop!(g, name, prop, value) = MG.set_prop!(g, g[name, :name], prop, value)
get_prop(g, name, prop) = MG.get_prop(g, g[name, :name], prop)


function setContact!(g, parentName, childName, contact::ShapeContact)
    parent = g[parentName, :name]
    child = g[childName, :name]
    LG.add_edge!(g, parent, child)
    setNamedEdgeProp!(g, (parentName, childName), :contact, contact)
end

getContact(g::LG.AbstractGraph, srcName, dstName) = getNamedEdgeProp(
        g, (srcName, dstName), :contact)

"""
Like `MetaGraphs.get_prop(g, edge, prop)`, but with the edge represented by a
pair of names `(srcName, dstName)` rather than a `LightGraphs.Edge`.
"""
function getNamedEdgeProp(g::MG.AbstractMetaGraph, namedEdge::Tuple, prop::Symbol)
    (srcName, dstName) = namedEdge
    e = _edgeFromNames(g, srcName, dstName)
    MG.get_prop(g, e, prop)
end

"""
Like `MetaGraphs.set_prop!(g, edge, prop, val)`, but with the edge represented
by a pair of names `(srcName, dstName)` rather than a `LightGraphs.Edge`.
"""
function setNamedEdgeProp!(g::MG.AbstractMetaGraph, namedEdge::Tuple,
                           prop::Symbol, val)
    (srcName, dstName) = namedEdge
    e = _edgeFromNames(g, srcName, dstName)
    MG.set_prop!(g, e, prop, val)
end

function _edgeFromNames(g::MG.AbstractMetaGraph, srcName, dstName)
    LG.Edge(g[srcName, :name], g[dstName, :name])
end

