# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros2_benchmark_interfaces:srv/StopMonitoring.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_StopMonitoring_Request(type):
    """Metaclass of message 'StopMonitoring_Request'."""

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
            module = import_type_support('ros2_benchmark_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2_benchmark_interfaces.srv.StopMonitoring_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__stop_monitoring__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__stop_monitoring__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__stop_monitoring__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__stop_monitoring__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__stop_monitoring__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class StopMonitoring_Request(metaclass=Metaclass_StopMonitoring_Request):
    """Message class 'StopMonitoring_Request'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

import builtins  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_StopMonitoring_Response(type):
    """Metaclass of message 'StopMonitoring_Response'."""

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
            module = import_type_support('ros2_benchmark_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2_benchmark_interfaces.srv.StopMonitoring_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__stop_monitoring__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__stop_monitoring__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__stop_monitoring__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__stop_monitoring__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__stop_monitoring__response

            from ros2_benchmark_interfaces.msg import TimestampedMessageArray
            if TimestampedMessageArray.__class__._TYPE_SUPPORT is None:
                TimestampedMessageArray.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class StopMonitoring_Response(metaclass=Metaclass_StopMonitoring_Response):
    """Message class 'StopMonitoring_Response'."""

    __slots__ = [
        '_start_timestamps',
        '_end_timestamps',
    ]

    _fields_and_field_types = {
        'start_timestamps': 'ros2_benchmark_interfaces/TimestampedMessageArray',
        'end_timestamps': 'ros2_benchmark_interfaces/TimestampedMessageArray',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['ros2_benchmark_interfaces', 'msg'], 'TimestampedMessageArray'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ros2_benchmark_interfaces', 'msg'], 'TimestampedMessageArray'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from ros2_benchmark_interfaces.msg import TimestampedMessageArray
        self.start_timestamps = kwargs.get('start_timestamps', TimestampedMessageArray())
        from ros2_benchmark_interfaces.msg import TimestampedMessageArray
        self.end_timestamps = kwargs.get('end_timestamps', TimestampedMessageArray())

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
        if self.start_timestamps != other.start_timestamps:
            return False
        if self.end_timestamps != other.end_timestamps:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def start_timestamps(self):
        """Message field 'start_timestamps'."""
        return self._start_timestamps

    @start_timestamps.setter
    def start_timestamps(self, value):
        if __debug__:
            from ros2_benchmark_interfaces.msg import TimestampedMessageArray
            assert \
                isinstance(value, TimestampedMessageArray), \
                "The 'start_timestamps' field must be a sub message of type 'TimestampedMessageArray'"
        self._start_timestamps = value

    @builtins.property
    def end_timestamps(self):
        """Message field 'end_timestamps'."""
        return self._end_timestamps

    @end_timestamps.setter
    def end_timestamps(self, value):
        if __debug__:
            from ros2_benchmark_interfaces.msg import TimestampedMessageArray
            assert \
                isinstance(value, TimestampedMessageArray), \
                "The 'end_timestamps' field must be a sub message of type 'TimestampedMessageArray'"
        self._end_timestamps = value


class Metaclass_StopMonitoring(type):
    """Metaclass of service 'StopMonitoring'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ros2_benchmark_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2_benchmark_interfaces.srv.StopMonitoring')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__stop_monitoring

            from ros2_benchmark_interfaces.srv import _stop_monitoring
            if _stop_monitoring.Metaclass_StopMonitoring_Request._TYPE_SUPPORT is None:
                _stop_monitoring.Metaclass_StopMonitoring_Request.__import_type_support__()
            if _stop_monitoring.Metaclass_StopMonitoring_Response._TYPE_SUPPORT is None:
                _stop_monitoring.Metaclass_StopMonitoring_Response.__import_type_support__()


class StopMonitoring(metaclass=Metaclass_StopMonitoring):
    from ros2_benchmark_interfaces.srv._stop_monitoring import StopMonitoring_Request as Request
    from ros2_benchmark_interfaces.srv._stop_monitoring import StopMonitoring_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
