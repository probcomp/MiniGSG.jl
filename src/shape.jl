abstract type Shape end

"""
`Shape` implementation for a box (rectangular prism).  The box's origin is at
its geometric center, and the faces of the box are axis-aligned (relative to
the coordinate frame of the box).

There are six contact plane families, each zero-dimensional and consisting of a
single contact plane:
* `:left` (`x = -sizeX / 2`)
* `:right` (`x = sizeX / 2`)
* `:bottom` (`z = -sizeZ / 2`)
* `:top` (`z = sizeZ / 2`)
* `:back` (`y = -sizeY / 2`)
* `:front` (`y = sizeY / 2`)

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
    if familyId == :top
        Pose(0, 0, sz, PC.IDENTITY_ORN)
    elseif familyId == :bottom
        Pose(0, 0, -sz, rotateAroundAxis([1, 0, 0], π))
    elseif familyId == :left
        Pose(-sx, 0, 0, rotateAroundAxis([0, 1, 0], -π/2))
    elseif familyId == :right
        Pose(sx, 0, 0, rotateAroundAxis([0, 1, 0], π/2))
    elseif familyId == :front
        Pose(0, sy, 0, rotateAroundAxis([1, 0, 0], -π/2))
    elseif familyId == :back
        Pose(0, -sy, 0, rotateAroundAxis([1, 0, 0], π/2))
    else
        error("Invalid contact plane family $familyId")
    end
end

const BOX_SURFACE_IDS = [:top, :bottom, :left, :right, :front, :back]
