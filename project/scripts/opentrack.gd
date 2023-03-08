extends Node


var pos: Vector3
var rpy: Vector3

var _joy_id := -1

func detect_opentrack(_device := -1, _connected := false):
    for id in Input.get_connected_joypads():
        if "opentrack headpose" == Input.get_joy_name(id):
            _joy_id = id


func _ready() -> void:
    Input.joy_connection_changed.connect(detect_opentrack)
    detect_opentrack()


func _process(_delta: float) -> void:
    if _joy_id == -1: return
    @warning_ignore("int_as_enum_without_cast")
    rpy.z = -Input.get_joy_axis(_joy_id, 3)
    @warning_ignore("int_as_enum_without_cast")
    rpy.y = -Input.get_joy_axis(_joy_id, 4)
    @warning_ignore("int_as_enum_without_cast")
    rpy.x = Input.get_joy_axis(_joy_id, 5)
