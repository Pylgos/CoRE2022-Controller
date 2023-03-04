extends Node


var _server := UDPServer.new()

var pos: Vector3
var rpy: Vector3

var _joy_id := -1

func detect_opentrack(_device := -1, _connected := false):
    for id in Input.get_connected_joypads():
        if "opentrack headpose" == Input.get_joy_name(id):
            _joy_id = id


func _ready() -> void:
    _server.listen(4242)
    Input.joy_connection_changed.connect(detect_opentrack)
    detect_opentrack()


func _process(_delta: float) -> void:
    if _joy_id == -1: return
    rpy.z = -Input.get_joy_axis(_joy_id, 3)
    rpy.y = -Input.get_joy_axis(_joy_id, 4)
    rpy.x = Input.get_joy_axis(_joy_id, 5)
    
#    _server.poll()
#    if _server.is_connection_available():
#        var peer := _server.take_connection()
#        var pkt := peer.get_packet()
#
#        if pkt.size() != 8 * 6:
#            push_error("Invalid packet size: ", pkt.size())
#            return
#
#        _buf.clear()
#        _buf.data_array = pkt
#
#        pos.x = _buf.get_double()
#        pos.y = _buf.get_double()
#        pos.z = _buf.get_double()
#
#        rpy.z = -deg_to_rad(_buf.get_double())
#        rpy.y = -deg_to_rad(_buf.get_double())
#        rpy.x = deg_to_rad(_buf.get_double())
    
#    print(rpy)
