@tool
extends MeshInstance3D
class_name Grid


@export var lines := 5 :
    set(value):
        lines = value
        update_mesh()
    get:
        return lines

@export var line_span := 1 :
    set(value):
        line_span = value
        update_mesh()
    get:
        return line_span


func _ready() -> void:
    mesh = ImmediateMesh.new()
    update_mesh()


func update_mesh() -> void:
    mesh.clear_surfaces()
    mesh.surface_begin(Mesh.PRIMITIVE_LINES)
    material_override = StandardMaterial3D.new()
    material_override.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
    material_override.albedo_color = Color(Color.DARK_GRAY, 0.5)
    material_override.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
    
    var length := lines * line_span
    var half_len := length / 2.0
    
    for i in lines:
        var x = i * line_span - half_len
        mesh.surface_add_vertex(Vector3(x, half_len, 0))
        mesh.surface_add_vertex(Vector3(x, -half_len, 0))
    
    for i in lines:
        var y = i * line_span - half_len
        mesh.surface_add_vertex(Vector3(half_len, y, 0))
        mesh.surface_add_vertex(Vector3(-half_len, y, 0))
    
    mesh.surface_end()
