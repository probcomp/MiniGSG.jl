module MiniGSG

import Base: @kwdef
import DataStructures: OrderedDict
import LightGraphs as LG
import MetaGraphs as MG
import PoseComposition: Pose
import PoseComposition as PC
import Rotations
import StaticArrays: SVector, StaticVector

NothingOr{T} = Union{Nothing, T}

function rotateAroundAxis(axis::StaticVector{3, <:Real}, angle::Real)
    Rotations.AngleAxis(angle, axis...)
end

function rotateAroundAxis(axis::AbstractVector{<:Real}, angle::Real)
    rotateAroundAxis(SVector{3}(axis), angle)
end

consecutivePairs(xs) = zip(xs, Iterators.drop(xs, 1))

include("planar_contact.jl")
include("shape.jl")
include("graph.jl")
include("graph_helpers.jl")

end  # module MiniGSG
