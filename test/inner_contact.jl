import MiniGSG as S
import PoseComposition as PC


obj_1 = S.Box(0.1, 0.3, 0.4)
container_1 = S.BoxContainer(0.2, 0.4, 0.1)
obj_2 = S.Box(0.3, 1.0, 0.6)

scene_graph = S.SceneGraph()
S.addObject!(scene_graph, :obj_1, obj_1)
S.addObject!(scene_graph, :container_1, container_1)
S.addObject!(scene_graph, :obj_2, obj_2)

S.setPose!(scene_graph, :obj_1, PC.IDENTITY_POSE)

contact = S.ShapeContact(:top, Float64[], :outer_bottom, Float64[],
    S.PlanarContact(0.0, 0.0, 0.0))
S.setContact!(scene_graph, :obj_1, :container_1, contact)

contact = S.ShapeContact(:inner_bottom, Float64[], :bottom, Float64[],
    S.PlanarContact(0.0, 0.0, 0.0))
S.setContact!(scene_graph, :container_1, :obj_2, contact)

poses = collect(values(S.floatingPosesOf(scene_graph)))

