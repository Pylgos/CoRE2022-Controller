@tool
extends MeshInstance3D


@export var length := 5 :
    set(value):
        length = value
        update_mesh()
    get:
        return length

@export var color := Color.GRAY :
    set(value):
        color = value
        update_mesh()
    get:
        return color

@export var fov := 75.0 :
    set(value):
        fov = value
        update_mesh()
    get:
        return fov


func _init() -> void:
    if mesh == null:
        mesh = ImmediateMesh.new()
    if material_override == null:
        material_override = StandardMaterial3D.new()

func _ready() -> void:
    update_mesh()
    Robot.camera_angle_changed.connect(update_mesh)

func update_mesh() -> void:
    mesh.clear_surfaces()
    mesh.surface_begin(Mesh.PRIMITIVE_LINES)
    material_override.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
    material_override.albedo_color = color
    material_override.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
    
    var yaw = Robot.get_camera_yaw()
    var theta_min = yaw - deg_to_rad(fov / 2)
    var theta_max = yaw + deg_to_rad(fov / 2)
    mesh.surface_add_vertex(Vector3.ZERO)
    mesh.surface_add_vertex(length * Vector3(cos(theta_min), sin(theta_min), 0))
    mesh.surface_add_vertex(Vector3.ZERO)
    mesh.surface_add_vertex(length * Vector3(cos(theta_max), sin(theta_max), 0))
    mesh.surface_end()
