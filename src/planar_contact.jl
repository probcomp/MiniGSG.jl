"""$(raw"""
3DoF representation of planar contact, including external tangency between
shapes as a special case.

## If used without a slack term

Represents the pose offset between two (right-handed) coordinate frames
whose `(x, y)` planes coincide but have opposite orientations.  In the case
where each of the two `(x, y)` planes is tangent to a surface and the positive
`z` directions are the outward normals to those surfaces, this corresponds to
the two surfaces being externally tangent.

Concretely, `PlanarContact(x, y, angle)` represents the following rigid motion:

    (Transform by `OUTWARD_NORMAL_FLIP`) ∘
    (Rotate by `angle` radians around the positive z-axis) ∘
    (Translate by `[x, y, 0]`)

Note that this is the usual right-associative function composition, meaning
that first the translation is done, then the rotation around the z-axis, then
the transformation by [`OUTWARD_NORMAL_FLIP`](@ref).

To obtain the 6DoF representation of a `PlanarContact`, use
[`planarContactTo6DOF`](@ref).

## If used with a slack term

The `slack` field allows us to specify inexact contacts, by specifying the
"error" between the pose offset implied by exact contact and the actual pose
offset.  Concretely, this means

    planarContactTo6DOF(PlanarContact(x, y, angle, slack::Pose)) ==
    planarContactTo6DOF(PlanarContact(x, y, angle)) * slack

See also: [`closestApproximatingContact`](@ref).
""")"""
@kwdef struct PlanarContact
    x::Real
    y::Real
    angle::Real
    slack::NothingOr{Pose} = nothing
end

PlanarContact(x::Real, y::Real, angle::Real) = PlanarContact(x=x, y=y,
                                                             angle=angle)

"""
Canonical choice of rigid motion that flips the z-axis.

This constant is part of the definition of what an "angular offset" is.
(Contact parameterizations are not required to include an angular offset
parameter, but in practice they usually do.)

Motivating use case: Suppose we have two surface patches `P1` (parent) and `P2`
(child), whose poses are `(pos1, orn1)` and `(pos2, orn2)` respectively, and
further suppose that the origin on each surface is also a point of tangency.
There is only one degree of freedom for describing the offset between these two
surface patches' coordinate frames, and conceptually we can think of it as an
angle of rotation around an axis that is perpendicular to the plane of
tangency.  This parameterization is not canonically defined, however, until we
choose a specific rigid motion that flips the z-axis.  Once we do that, the
parameterization is defined as follows:

Note that because the tangency is external, the outward normals to the two
surfaces at the point of tangency (that is, the `z`-axes of `orn1` and `orn2`)
point in opposite directions.  Thus, letting `orn3 = OUTWARD_NORMAL_FLIP *
orn2`, we have that the `z`-axes of `orn1` and `orn3` point in the same
direction, and their `(x, y)` planes coincide and have the same orientation.
Therefore there exists an angle `θ`, unique up to an integer multiple of 2π,
such that `orn3 = rotateAroundAxis([0, 0, 1], θ) * orn1` (in words, `orn3` is
equal to `orn1` rotated around the common `z`-axis by `θ` radians).  We define
this angle `θ` to be the angular offset between `P1` and `P2`.
"""
const OUTWARD_NORMAL_FLIP = rotateAroundAxis([1, 1, 0], π)


"""
Contact relation between 3D `Shape`s.

That is, a choice of contact plane on each shape, and a `PlanarContact`
specifying the relative pose of the contact plane on the child shape relative
to the contact plane on the parent shape.
"""
@kwdef struct ShapeContact
    """ID of the contact plane family on the parent shape."""
    parentFamilyId::Any
    """Parameters specifying the contact plane on the parent shape within its
    contact plane family (as in `getContactPlane`)."""
    parentPlaneParams::Vector{<:Real}
    """ID of the contact plane family on the child shape."""
    childFamilyId::Any
    """Parameters specifying the contact plane on the child shape within its
    contact plane family (as in `getContactPlane`)."""
    childPlaneParams::Vector{<:Real}
    """
    3DoF Relative pose of the child contact plane relative to the parent contact
    plane.  We have 3DoF here because we know the two contact planes are
    externally tangent.
    """
    planarContact::PlanarContact
end


"""
Expresses a 6DOF `Pose` as a 3DOF `PlanarContact` with the "smallest possible"
slack term.  That is,

    pose == planarContactTo6DOF(closestApproximatingContact(pose))

The `contact.slack` is chosen to have the smallest possible translational
offset (in fact, `contact.slack.pos == [0, 0, z]` for some `z`), and the
smallest cosine distance from its orientation to the identity.

Note in particular that if `pose` can be gotten from an _exact_ contact (i.e.,
one with `contact.slack == IDENTITY_POSE`), then this function will find that
exact contact.
"""
function closestApproximatingContact(pose::Pose)::PlanarContact
    # The `contact` we choose will satisfy
    #
    #     pose = planarContactTo6DOF(contact)
    #          = Pose([0, 0, 0], OUTWARD_NORMAL_FLIP)
    #                * rotateAroundAxis([0, 0, 1], contact.angle)
    #                * contact.slack
    #
    # Hence
    #
    #    rotateAroundAxis([0, 0, 1], contact.angle) =
    #        Pose([0, 0, 0], OUTWARD_NORMAL_FLIP)
    #            \ pose
    #            / contact.slack
    #
    # Note that minimizing the cosine distance between the identity and
    # `contact.slack` is equivalent to minimizing the cosine distance between the
    # identity and `inv(contact.slack)`, which in turn is equivalent to
    # minimizing the cosine distance between `Pose([0, 0, 0],
    # OUTWARD_NORMAL_FLIP) \ pose` and `rotateAroundAxis([1, 0, 0],
    # contact.angle)`.
    (x, y, _) = pose.pos
    pose_ = Pose([0, 0, 0], OUTWARD_NORMAL_FLIP) \ pose
    q = Rotations.UnitQuaternion(pose_.orientation)
    ### Derivation of the optimal  `PlanarContact.angle`
    #
    # We want to maximize the cosine between `orn` and
    # `rotateAroundAxis([1, 0, 0], contact.angle)`.  Abbreviate `contact.angle`
    # as `θ`, and note that
    #
    #     rotateAroundAxis([1, 0, 0], θ) == UnitQuaternion(cos(θ/2), 0, 0, sin(θ/2))
    #
    # By [1], this cosine is
    #
    #     2 * DotProduct(± UnitQuaternion(cos(θ/2), 0, 0, sin(θ/2)), orn)^2 - 1
    #
    # This is maximized precisely when the `DotProduct` is either maximized or
    # minimized, so we can drop the ±.  We have
    #
    #     DotProduct((cos(θ/2), 0, 0, sin(θ/2)),
    #                componentsWXYZ(UnitQuaternion(orn)))
    #         = cos(θ/2) * Re(orn) + sin(θ/2) * Im_3(orn)
    # 
    # And, by calculus (or by clever right triangle trigonometry), that is
    # minimized or maximized when
    #
    #     θ/2 = arctan(Im_3(orn) / Re(orn)) + π * n for some n ∈ ℤ,
    #
    # where `Im_3` denotes the third imaginary component, or "k" component, of
    # `orn`; equivalently,
    #
    #     θ = 2 * arctan(Im_3(orn) / Re(orn)) + 2π * n for some n ∈ ℤ.
    #
    # So the "best" value of θ, which is unique up to a multiple of 2π, is
    #
    #     θ = 2 * arctan(Im_3(orn) / Re(orn))
    #
    # Or in Rotations.jl notation,
    #
    #     θ = 2 * atan(UnitQuaternion(orn).z, UnitQuaternion(orn).w)
    #
    # [1] https://math.stackexchange.com/questions/90081/quaternion-distance
    angle = 2 * atan(q.z, q.w)

    slack = planarContactTo6DOF(PlanarContact(x, y, angle)) \ pose
    return PlanarContact(x, y, angle, slack)
end


"""
Converts a 3DoF planar contact (external tangency) to 6DoF relative pose of the
child plane's coordinate frame relative to the parent plane's coordinate frame.
"""
function planarContactTo6DOF(c::PlanarContact)::Pose
    p = Pose(c.x, c.y, 0, OUTWARD_NORMAL_FLIP * rotateAroundAxis([0, 0, 1],
                                                                 c.angle))
    if c.slack |> isnothing
        p
    else
        p * c.slack
    end
end
