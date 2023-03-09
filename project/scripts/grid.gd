@tool
extends MeshInstance3D
class_name Grid


enum Mode {
    POLAR,
    RECTANGULAR,
}


@export var mode := Mode.RECTANGULAR :
    set(value):
        mode = value
        update_mesh()
    get:
        return mode

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

@export var color := Color.GRAY :
    set(value):
        color = value
        update_mesh()
    get:
        return color


func _ready() -> void:
    if mesh == null:
        mesh = ImmediateMesh.new()
    
    if material_override == null:
        material_override = StandardMaterial3D.new()
    
    update_mesh()


func update_mesh() -> void:
    mesh.clear_surfaces()
    mesh.surface_begin(Mesh.PRIMITIVE_LINES)
    material_override.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
    material_override.albedo_color = color
    material_override.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
    
    match mode:
        Mode.RECTANGULAR:
            var length := lines * line_span
            var half_len := length / 2.0
            
            for i in lines + 1:
                var x = i * line_span - half_len
                mesh.surface_add_vertex(Vector3(x, half_len, 0))
                mesh.surface_add_vertex(Vector3(x, -half_len, 0))
            
            for i in lines + 1:
                var y = i * line_span - half_len
                mesh.surface_add_vertex(Vector3(half_len, y, 0))
                mesh.surface_add_vertex(Vector3(-half_len, y, 0))

        Mode.POLAR:
            const div := 180
            for i in lines:
                var r = (i + 1) * line_span
                for j in div:
                    var angle = j * TAU / div
                    mesh.surface_add_vertex(Vector3(r * cos(angle), r * sin(angle), 0))
                    mesh.surface_add_vertex(Vector3(r * cos(angle + TAU / div), r * sin(angle + TAU / div), 0))
            
            const angle_div := 8
            for i in angle_div:
                var angle = i * TAU / angle_div
                var r = lines * line_span
                mesh.surface_add_vertex(Vector3.ZERO)
                mesh.surface_add_vertex(Vector3(r * cos(angle), r * sin(angle), 0))

    mesh.surface_end()
