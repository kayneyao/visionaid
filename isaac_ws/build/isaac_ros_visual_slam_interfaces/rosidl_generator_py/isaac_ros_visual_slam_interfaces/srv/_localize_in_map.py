# generated from rosidl_generator_py/resource/_idl.py.em
# with input from isaac_ros_visual_slam_interfaces:srv/LocalizeInMap.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LocalizeInMap_Request(type):
    """Metaclass of message 'LocalizeInMap_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('isaac_ros_visual_slam_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'isaac_ros_visual_slam_interfaces.srv.LocalizeInMap_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__localize_in_map__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__localize_in_map__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__localize_in_map__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__localize_in_map__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__localize_in_map__request

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LocalizeInMap_Request(metaclass=Metaclass_LocalizeInMap_Request):
    """Message class 'LocalizeInMap_Request'."""

    __slots__ = [
        '_map_folder_path',
        '_pose_hint',
    ]

    _fields_and_field_types = {
        'map_folder_path': 'string',
        'pose_hint': 'geometry_msgs/Pose',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.map_folder_path = kwargs.get('map_folder_path', str())
        from geometry_msgs.msg import Pose
        self.pose_hint = kwargs.get('pose_hint', Pose())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.map_folder_path != other.map_folder_path:
            return False
        if self.pose_hint != other.pose_hint:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def map_folder_path(self):
        """Message field 'map_folder_path'."""
        return self._map_folder_path

    @map_folder_path.setter
    def map_folder_path(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'map_folder_path' field must be of type 'str'"
        self._map_folder_path = value

    @builtins.property
    def pose_hint(self):
        """Message field 'pose_hint'."""
        return self._pose_hint

    @pose_hint.setter
    def pose_hint(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'pose_hint' field must be a sub message of type 'Pose'"
        self._pose_hint = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_LocalizeInMap_Response(type):
    """Metaclass of message 'LocalizeInMap_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('isaac_ros_visual_slam_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'isaac_ros_visual_slam_interfaces.srv.LocalizeInMap_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__localize_in_map__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__localize_in_map__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__localize_in_map__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__localize_in_map__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__localize_in_map__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LocalizeInMap_Response(metaclass=Metaclass_LocalizeInMap_Response):
    """Message class 'LocalizeInMap_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value


class Metaclass_LocalizeInMap(type):
    """Metaclass of service 'LocalizeInMap'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('isaac_ros_visual_slam_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'isaac_ros_visual_slam_interfaces.srv.LocalizeInMap')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__localize_in_map

            from isaac_ros_visual_slam_interfaces.srv import _localize_in_map
            if _localize_in_map.Metaclass_LocalizeInMap_Request._TYPE_SUPPORT is None:
                _localize_in_map.Metaclass_LocalizeInMap_Request.__import_type_support__()
            if _localize_in_map.Metaclass_LocalizeInMap_Response._TYPE_SUPPORT is None:
                _localize_in_map.Metaclass_LocalizeInMap_Response.__import_type_support__()


class LocalizeInMap(metaclass=Metaclass_LocalizeInMap):
    from isaac_ros_visual_slam_interfaces.srv._localize_in_map import LocalizeInMap_Request as Request
    from isaac_ros_visual_slam_interfaces.srv._localize_in_map import LocalizeInMap_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
