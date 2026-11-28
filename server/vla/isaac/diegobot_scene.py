"""Isaac Lab scene pieces of diegobot at its workplace, with the head and wrist cameras placed as on the real robot.

Import only after the simulation app has started (``launch_simulation``).
"""

import math
import os
import re

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
# The converter keeps every URDF link as a nested prim, so links are given by their full path.
BASE = '{ENV_REGEX_NS}/Robot/Geometry/base_footprint/base_link'
GRIPPER_LINK = (f'{BASE}/arm_mount_link/arm_base_link/shoulder_link/upper_arm_link/lower_arm_link/wrist_link/'
                'gripper_link')
HEAD_CAMERA_LINK = f'{BASE}/head_base_link/head_pan_link/head_tilt_link/head_camera_link'
WRIST_CAMERA_LINK = f'{GRIPPER_LINK}/arm_camera_mount_link/hp60c_link/hp60c_calib_link/hp60c_upright_link'
# Head tilt the episodes were recorded and run with.
HEAD_TILT = -0.86
# Arm pose the real episodes start from (the policy runner's home pose).
HOME_JOINTS = (0.0, -1.6, 1.33, 1.39, -1.46, 0.0)
BLOCK_SIZE = (0.06, 0.03, 0.03)

# Real-robot looks by mesh, each (linear colour, roughness, specular colour level): the printed SO-101 parts are white,
# the plates mounting the arm and the head to the chassis ivory, the printed head arm and the arm's driver board
# plate orange, the wrist camera silver, the rest (servos, chassis, head camera) black.
LOOKS = {
    'arm_white': ((0.82, 0.82, 0.80), 0.9, 0.02),
    # Ivory mounting plates; linear RGB of about sRGB (238, 231, 216).
    'ivory': ((0.85, 0.80, 0.69), 0.9, 0.02),
    # Bambu Lab PLA Tough+ Orange (12301), sRGB #DC3A27 in linear RGB.
    'orange': ((0.716, 0.042, 0.020), 0.6, 0.04),
    # Brushed-metal look of the wrist camera's casing: grey base with a strong, slightly blurred reflection.
    'silver': ((0.40, 0.40, 0.41), 0.35, 0.55),
    # Slightly glossy like the real plastic, so edges and screw heads show instead of a flat silhouette.
    'part_black': ((0.02, 0.02, 0.02), 0.35, 0.06),
}
# Linear RGB of the real block's blue.
BLOCK_BLUE = (0.012, 0.12, 0.70)


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def camera(parent, name, k, width, height, pos, rot):
    # Offsets are the camera drivers' static TF from the URDF link to the colour optical frame.
    return CameraCfg(
        prim_path=f'{parent}/{name}',
        update_period=0.0,
        width=width,
        height=height,
        data_types=['rgb'],
        spawn=sim_utils.PinholeCameraCfg.from_intrinsic_matrix(
            intrinsic_matrix=k, width=width, height=height, clipping_range=(0.01, 20.0)),
        offset=CameraCfg.OffsetCfg(pos=pos, rot=rot, convention='ros'),
    )


def head_camera():
    return camera(HEAD_CAMERA_LINK, 'head_cam', [406.371, 0.0, 419.607, 0.0, 406.475, 237.234, 0.0, 0.0, 1.0],
                  848, 480, (0.0, -0.024, 0.0), (0.5, -0.5, 0.5, -0.5))


def wrist_camera():
    return camera(WRIST_CAMERA_LINK, 'wrist_cam', [594.288, 0.0, 301.633, 0.0, 593.767, 240.482, 0.0, 0.0, 1.0],
                  640, 480, (0.002, -0.012, 0.0), (-0.501, 0.502, -0.496, 0.501))


def robot_cfg(usd, joints=HOME_JOINTS):
    return ArticulationCfg(
        prim_path='{ENV_REGEX_NS}/Robot',
        spawn=sim_utils.UsdFileCfg(usd_path=os.path.abspath(usd)),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={**dict(zip(ARM_JOINTS, joints)), 'head_pan': 0.0, 'head_tilt': HEAD_TILT, 'wheel_.*': 0.0}),
        actuators={
            'arm': ImplicitActuatorCfg(joint_names_expr=ARM_JOINTS, stiffness=400.0, damping=20.0),
            # The head only holds a fixed pose; a softer drive lets it sag and shifts the head camera view.
            'head': ImplicitActuatorCfg(joint_names_expr=['head_pan', 'head_tilt'], stiffness=5000.0, damping=100.0),
            'wheels': ImplicitActuatorCfg(joint_names_expr=['wheel_.*'], stiffness=0.0, damping=10.0),
        },
    )


def block_cfg(x=0.25, y=0.10, yaw=0.0, ghost=False):
    return RigidObjectCfg(
        prim_path='{ENV_REGEX_NS}/Block',
        spawn=sim_utils.CuboidCfg(
            size=BLOCK_SIZE,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=ghost),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.02),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=not ghost),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=BLOCK_BLUE),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(x, y, BLOCK_SIZE[2] / 2), rot=yaw_quat(yaw)),
    )


@configclass
class DiegobotSceneCfg(InteractiveSceneCfg):
    """Robot, block and cameras; ``robot`` has to be filled in with :func:`robot_cfg`."""

    # The ground plane only collides; the floor look comes from the textured mesh made in make_floor.
    ground = AssetBaseCfg(prim_path='/World/ground', spawn=sim_utils.GroundPlaneCfg(visible=False))
    dome_light = AssetBaseCfg(prim_path='/World/dome_light',
                              spawn=sim_utils.DomeLightCfg(intensity=600.0, color=(1.0, 0.97, 0.92)))
    # Ceiling light: a directional light from above and slightly in front gives the shading the dome light lacks.
    ceiling_light = AssetBaseCfg(prim_path='/World/ceiling_light',
                                 spawn=sim_utils.DistantLightCfg(intensity=0.0, angle=20.0, color=(1.0, 0.97, 0.92)),
                                 init_state=AssetBaseCfg.InitialStateCfg(rot=(0.0, 0.2588, 0.0, 0.9659)))
    robot: ArticulationCfg = None
    block = block_cfg()
    head_camera = head_camera()
    wrist_camera = wrist_camera()


def make_floor(stage, path, texture, tile, yaw=0.0, size=6.0):
    """Textured square floor at z=0. Its UVs repeat the image every ``tile`` metres."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    h, n = size / 2, size / tile
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([(-h, -h, 0.0), (h, -h, 0.0), (h, h, 0.0), (-h, h, 0.0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([(0.0, 0.0, 1.0)] * 4)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([(0.0, 0.0), (n, 0.0), (n, n), (0.0, n)])

    material = UsdShade.Material.Define(stage, f'{path}_material')
    shader = UsdShade.Shader.Define(stage, f'{path}_material/surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.55)
    reader = UsdShade.Shader.Define(stage, f'{path}_material/st_reader')
    reader.CreateIdAttr('UsdPrimvarReader_float2')
    reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
    image = UsdShade.Shader.Define(stage, f'{path}_material/image')
    image.CreateIdAttr('UsdUVTexture')
    image.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(texture)
    image.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('repeat')
    image.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('repeat')
    image.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), 'result')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(image.ConnectableAPI(), 'rgb')
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    UsdGeom.XformCommonAPI(mesh).SetRotate(Gf.Vec3f(0.0, 0.0, math.degrees(yaw)))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    return image.GetInput('file')


def floor_texture(folder, name):
    return next(os.path.abspath(os.path.join(folder, f)) for f in sorted(os.listdir(folder))
                if os.path.splitext(f)[0] == name)


def make_looks(stage):
    from pxr import Gf, Sdf, UsdShade

    for name, (color, roughness, specular) in LOOKS.items():
        sim_utils.spawn_preview_surface(f'/World/Looks/{name}',
                                        sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=roughness))
        # The default dielectric reflection adds a grey sheen that turns black parts mid-grey under the dome light.
        shader = UsdShade.Shader(stage.GetPrimAtPath(f'/World/Looks/{name}/Shader'))
        shader.CreateInput('useSpecularWorkflow', Sdf.ValueTypeNames.Int).Set(1)
        shader.CreateInput('specularColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(specular, specular, specular))


IVORY_MESHES = ('arm_mount', 'base_chassis_linker')
SILVER_MESHES = ('camera_NUWA_HP60C',)
ORANGE_MESHES = ('base', 'pan_base_linker', 'pan_arm_bottom', 'pan_arm_pillar', 'pan_arm_top', 'tilt_arm',
                 'waveshare_mounting_plate_so101_v2')


def look_of(mesh_holder):
    # The converter appends _1, _2 ... when one link uses the same mesh twice.
    name = re.sub(r'_\d+$', '', mesh_holder)
    if name in IVORY_MESHES:
        return 'ivory'
    if name in ORANGE_MESHES:
        return 'orange'
    if name in SILVER_MESHES:
        return 'silver'
    if 'so101' in name:
        return 'arm_white'
    return 'part_black'


def paint_robot(stage, robot_path):
    import isaaclab.sim.utils.prims as prim_utils
    from pxr import Usd

    # Each URDF visual becomes an Xform named after its mesh file under the link Xform that owns it.
    for prim in list(Usd.PrimRange(stage.GetPrimAtPath(f'{robot_path}/Geometry'))):
        mesh_holder = prim.GetName()
        if prim.GetTypeName() != 'Xform' or mesh_holder.endswith('_link') or mesh_holder == 'base_footprint':
            continue
        # Visuals are instanceable references whose meshes carry their own URDF colours; un-instance to override.
        prim.SetInstanceable(False)
        look = look_of(mesh_holder)
        prim_utils.bind_visual_material(prim.GetPath(), f'/World/Looks/{look}', stage=stage)


def dress_scene(stage, num_envs, texture, tile, yaw):
    """Real-robot colours and the textured floor; the robot USD and the ground plane alone look different."""
    make_looks(stage)
    for i in range(num_envs):
        paint_robot(stage, f'/World/envs/env_{i}/Robot')
    make_floor(stage, '/World/floor', texture, tile, yaw)
