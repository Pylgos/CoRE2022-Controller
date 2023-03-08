@tool
class_name Ruler
extends MeshInstance3D


@export var length := 5 :
    set(value):
        length = value
        update_mesh()
    get:
        return length

@export var increment := 1 :
    set(value):
        increment = value
        update_mesh()
    get:
        return increment

@export var tick_length := 0.5 :
    set(value):
        tick_length = value
        update_mesh()
    get:
        return tick_length

@export var color := Color.GRAY :
    set(value):
        color = value
        update_mesh()
    get:
        return color


func _init() -> void:
    mesh = ImmediateMesh.new()


func _ready() -> void:
    update_mesh()


func update_mesh() -> void:
    mesh.clear_surfaces()
    mesh.surface_begin(Mesh.PRIMITIVE_LINES)
    material_override = StandardMaterial3D.new()
    material_override.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
    material_override.albedo_color = color
    material_override.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
    
    @warning_ignore("integer_division")
    var tick_count := length / increment
    
    for i in tick_count:
        var x = increment * (i + 1)
        mesh.surface_add_vertex(Vector3(x, tick_length / 2, 0))
        mesh.surface_add_vertex(Vector3(x, -tick_length / 2, 0))
    
    mesh.surface_add_vertex(Vector3(0, 0, 0))
    mesh.surface_add_vertex(Vector3(length, 0, 0))
    
    mesh.surface_end()
