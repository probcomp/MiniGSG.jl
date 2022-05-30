abstract type Shape end

"""
`Shape` implementation for a box (rectangular prism).  The box's origin is at
its geometric center, and the faces of the box are axis-aligned (relative to
the coordinate frame of the box).

There are six contact plane families, each zero-dimensional and consisting of a
single contact plane:



Each contact plane's origin is at the center of the corresponding face.  For
the orientations assigned to these contact planes, see the implementation of
[`getContactPlane(::Box, ::Symbol)`](@ref).  (Unfortunately it seems that there
is no fully canonical choice of how to assign the orientations, so it's left as
a hard-coded list.)
"""
@kwdef struct Box <: Shape
    sizeX::Real
    sizeY::Real
    sizeZ::Real
end

function getContactPlane(box::Box, familyId::Symbol)::Pose
    sx = box.sizeX / 2
    sy = box.sizeY / 2
    sz = box.sizeZ / 2
    if familyId == :back
        Pose(0, 0, sz, PC.IDENTITY_ORN)
    elseif familyId == :front
        Pose(0, 0, -sz, rotateAroundAxis([1, 0, 0], π))
    elseif familyId == :left
        Pose(-sx, 0, 0, rotateAroundAxis([0, 1, 0], -π/2))
    elseif familyId == :right
        Pose(sx, 0, 0, rotateAroundAxis([0, 1, 0], π/2))
    elseif familyId == :bottom
        Pose(0, sy, 0, rotateAroundAxis([1, 0, 0], -π/2))
    elseif familyId == :top
        Pose(0, -sy, 0, rotateAroundAxis([1, 0, 0], π/2))
    else
        error("Invalid contact plane family $familyId")
    end
end

const BOX_SURFACE_IDS = [:top, :bottom, :left, :right, :front, :back]

@kwdef struct BoxContainer <: Shape
    sizeX::Real
    sizeY::Real
    sizeZ::Real
end

function getContactPlane(box::BoxContainer, familyId::Symbol)::Pose
    sx = box.sizeX / 2
    sy = box.sizeY / 2
    sz = box.sizeZ / 2

    if familyId == :outer_back
        Pose(0, 0, sz, PC.IDENTITY_ORN)
    elseif familyId == :inner_back
        Pose(0, 0, sz, rotateAroundAxis([1, 0, 0], π))

    elseif familyId == :outer_front
        Pose(0, 0, -sz, rotateAroundAxis([1, 0, 0], π))
    elseif familyId == :inner_front
        Pose(0, 0, -sz, PC.IDENTITY_ORN)

    elseif familyId == :outer_left
        Pose(-sx, 0, 0, rotateAroundAxis([0, 1, 0], -π/2))
    elseif familyId == :inner_left
        Pose(-sx, 0, 0, rotateAroundAxis([0, 1, 0], π/2))

    elseif familyId == :outer_right
        Pose(sx, 0, 0, rotateAroundAxis([0, 1, 0], π/2))
    elseif familyId == :inner_right
        Pose(sx, 0, 0, rotateAroundAxis([0, 1, 0], -π/2))

    elseif familyId == :outer_bottom
        Pose(0, sy, 0, rotateAroundAxis([1, 0, 0], -π/2))
    elseif familyId == :inner_bottom
        Pose(0, sy, 0, rotateAroundAxis([1, 0, 0], π/2))


    elseif familyId == :outer_top
        Pose(0, -sy, 0, rotateAroundAxis([1, 0, 0], π/2))
    elseif familyId == :inner_top
        Pose(0, -sy, 0, rotateAroundAxis([1, 0, 0], -π/2))

    else
        error("Invalid contact plane family $familyId")
    end
end

const BOX_CONTAINER_SURFACE_IDS = [
    :outer_top, :outer_bottom, :outer_left, :outer_right, :outer_front, :outer_back,
    :inner_top, :inner_bottom, :inner_left, :inner_right, :inner_front, :inner_back,
]

