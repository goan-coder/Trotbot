# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from navigation/PlannerRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import navigation.msg

class PlannerRequest(genpy.Message):
  _md5sum = "312cafca219eff06ca8155401fd152ea"
  _type = "navigation/PlannerRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """navigation/Point_xy start
navigation/Point_xy goal
navigation/PolyArray obstacle_list

================================================================================
MSG: navigation/Point_xy
float32[] point
================================================================================
MSG: navigation/PolyArray
navigation/PointArray[] polygons

================================================================================
MSG: navigation/PointArray
navigation/Point_xy[] points
  """
  __slots__ = ['start','goal','obstacle_list']
  _slot_types = ['navigation/Point_xy','navigation/Point_xy','navigation/PolyArray']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       start,goal,obstacle_list

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PlannerRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.start is None:
        self.start = navigation.msg.Point_xy()
      if self.goal is None:
        self.goal = navigation.msg.Point_xy()
      if self.obstacle_list is None:
        self.obstacle_list = navigation.msg.PolyArray()
    else:
      self.start = navigation.msg.Point_xy()
      self.goal = navigation.msg.Point_xy()
      self.obstacle_list = navigation.msg.PolyArray()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      length = len(self.start.point)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.start.point))
      length = len(self.goal.point)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.goal.point))
      length = len(self.obstacle_list.polygons)
      buff.write(_struct_I.pack(length))
      for val1 in self.obstacle_list.polygons:
        length = len(val1.points)
        buff.write(_struct_I.pack(length))
        for val2 in val1.points:
          length = len(val2.point)
          buff.write(_struct_I.pack(length))
          pattern = '<%sf'%length
          buff.write(struct.pack(pattern, *val2.point))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.start is None:
        self.start = navigation.msg.Point_xy()
      if self.goal is None:
        self.goal = navigation.msg.Point_xy()
      if self.obstacle_list is None:
        self.obstacle_list = navigation.msg.PolyArray()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.start.point = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.goal.point = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.obstacle_list.polygons = []
      for i in range(0, length):
        val1 = navigation.msg.PointArray()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.points = []
        for i in range(0, length):
          val2 = navigation.msg.Point_xy()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sf'%length
          start = end
          end += struct.calcsize(pattern)
          val2.point = struct.unpack(pattern, str[start:end])
          val1.points.append(val2)
        self.obstacle_list.polygons.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.start.point)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.start.point.tostring())
      length = len(self.goal.point)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.goal.point.tostring())
      length = len(self.obstacle_list.polygons)
      buff.write(_struct_I.pack(length))
      for val1 in self.obstacle_list.polygons:
        length = len(val1.points)
        buff.write(_struct_I.pack(length))
        for val2 in val1.points:
          length = len(val2.point)
          buff.write(_struct_I.pack(length))
          pattern = '<%sf'%length
          buff.write(val2.point.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.start is None:
        self.start = navigation.msg.Point_xy()
      if self.goal is None:
        self.goal = navigation.msg.Point_xy()
      if self.obstacle_list is None:
        self.obstacle_list = navigation.msg.PolyArray()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.start.point = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.goal.point = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.obstacle_list.polygons = []
      for i in range(0, length):
        val1 = navigation.msg.PointArray()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.points = []
        for i in range(0, length):
          val2 = navigation.msg.Point_xy()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sf'%length
          start = end
          end += struct.calcsize(pattern)
          val2.point = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
          val1.points.append(val2)
        self.obstacle_list.polygons.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from navigation/PlannerResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import navigation.msg

class PlannerResponse(genpy.Message):
  _md5sum = "d2ffe07356360c7bae31566a65032850"
  _type = "navigation/PlannerResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """navigation/PointArray path
bool ack

================================================================================
MSG: navigation/PointArray
navigation/Point_xy[] points
  
================================================================================
MSG: navigation/Point_xy
float32[] point"""
  __slots__ = ['path','ack']
  _slot_types = ['navigation/PointArray','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       path,ack

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PlannerResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.path is None:
        self.path = navigation.msg.PointArray()
      if self.ack is None:
        self.ack = False
    else:
      self.path = navigation.msg.PointArray()
      self.ack = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      length = len(self.path.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.path.points:
        length = len(val1.point)
        buff.write(_struct_I.pack(length))
        pattern = '<%sf'%length
        buff.write(struct.pack(pattern, *val1.point))
      buff.write(_get_struct_B().pack(self.ack))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.path is None:
        self.path = navigation.msg.PointArray()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.path.points = []
      for i in range(0, length):
        val1 = navigation.msg.Point_xy()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sf'%length
        start = end
        end += struct.calcsize(pattern)
        val1.point = struct.unpack(pattern, str[start:end])
        self.path.points.append(val1)
      start = end
      end += 1
      (self.ack,) = _get_struct_B().unpack(str[start:end])
      self.ack = bool(self.ack)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.path.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.path.points:
        length = len(val1.point)
        buff.write(_struct_I.pack(length))
        pattern = '<%sf'%length
        buff.write(val1.point.tostring())
      buff.write(_get_struct_B().pack(self.ack))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.path is None:
        self.path = navigation.msg.PointArray()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.path.points = []
      for i in range(0, length):
        val1 = navigation.msg.Point_xy()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sf'%length
        start = end
        end += struct.calcsize(pattern)
        val1.point = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
        self.path.points.append(val1)
      start = end
      end += 1
      (self.ack,) = _get_struct_B().unpack(str[start:end])
      self.ack = bool(self.ack)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
class Planner(object):
  _type          = 'navigation/Planner'
  _md5sum = '0f8ba09d5a21e9916f0e3bf633247872'
  _request_class  = PlannerRequest
  _response_class = PlannerResponse
