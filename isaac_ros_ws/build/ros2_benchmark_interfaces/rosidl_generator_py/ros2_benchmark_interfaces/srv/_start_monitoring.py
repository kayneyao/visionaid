# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros2_benchmark_interfaces:srv/StartMonitoring.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_StartMonitoring_Request(type):
    """Metaclass of message 'StartMonitoring_Request'."""

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
                'ros2_benchmark_interfaces.srv.StartMonitoring_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__start_monitoring__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__start_monitoring__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__start_monitoring__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__start_monitoring__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__start_monitoring__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'REVISE_TIMESTAMPS_AS_MESSAGE_IDS__DEFAULT': False,
            'RECORD_START_TIMESTAMPS__DEFAULT': False,
        }

    @property
    def REVISE_TIMESTAMPS_AS_MESSAGE_IDS__DEFAULT(cls):
        """Return default value for message field 'revise_timestamps_as_message_ids'."""
        return False

    @property
    def RECORD_START_TIMESTAMPS__DEFAULT(cls):
        """Return default value for message field 'record_start_timestamps'."""
        return False


class StartMonitoring_Request(metaclass=Metaclass_StartMonitoring_Request):
    """Message class 'StartMonitoring_Request'."""

    __slots__ = [
        '_message_count',
        '_revise_timestamps_as_message_ids',
        '_record_start_timestamps',
    ]

    _fields_and_field_types = {
        'message_count': 'uint64',
        'revise_timestamps_as_message_ids': 'boolean',
        'record_start_timestamps': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.message_count = kwargs.get('message_count', int())
        self.revise_timestamps_as_message_ids = kwargs.get(
            'revise_timestamps_as_message_ids', StartMonitoring_Request.REVISE_TIMESTAMPS_AS_MESSAGE_IDS__DEFAULT)
        self.record_start_timestamps = kwargs.get(
            'record_start_timestamps', StartMonitoring_Request.RECORD_START_TIMESTAMPS__DEFAULT)

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
        if self.message_count != other.message_count:
            return False
        if self.revise_timestamps_as_message_ids != other.revise_timestamps_as_message_ids:
            return False
        if self.record_start_timestamps != other.record_start_timestamps:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def message_count(self):
        """Message field 'message_count'."""
        return self._message_count

    @message_count.setter
    def message_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'message_count' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'message_count' field must be an unsigned integer in [0, 18446744073709551615]"
        self._message_count = value

    @builtins.property
    def revise_timestamps_as_message_ids(self):
        """Message field 'revise_timestamps_as_message_ids'."""
        return self._revise_timestamps_as_message_ids

    @revise_timestamps_as_message_ids.setter
    def revise_timestamps_as_message_ids(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'revise_timestamps_as_message_ids' field must be of type 'bool'"
        self._revise_timestamps_as_message_ids = value

    @builtins.property
    def record_start_timestamps(self):
        """Message field 'record_start_timestamps'."""
        return self._record_start_timestamps

    @record_start_timestamps.setter
    def record_start_timestamps(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'record_start_timestamps' field must be of type 'bool'"
        self._record_start_timestamps = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_StartMonitoring_Response(type):
    """Metaclass of message 'StartMonitoring_Response'."""

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
                'ros2_benchmark_interfaces.srv.StartMonitoring_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__start_monitoring__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__start_monitoring__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__start_monitoring__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__start_monitoring__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__start_monitoring__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class StartMonitoring_Response(metaclass=Metaclass_StartMonitoring_Response):
    """Message class 'StartMonitoring_Response'."""

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


class Metaclass_StartMonitoring(type):
    """Metaclass of service 'StartMonitoring'."""

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
                'ros2_benchmark_interfaces.srv.StartMonitoring')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__start_monitoring

            from ros2_benchmark_interfaces.srv import _start_monitoring
            if _start_monitoring.Metaclass_StartMonitoring_Request._TYPE_SUPPORT is None:
                _start_monitoring.Metaclass_StartMonitoring_Request.__import_type_support__()
            if _start_monitoring.Metaclass_StartMonitoring_Response._TYPE_SUPPORT is None:
                _start_monitoring.Metaclass_StartMonitoring_Response.__import_type_support__()


class StartMonitoring(metaclass=Metaclass_StartMonitoring):
    from ros2_benchmark_interfaces.srv._start_monitoring import StartMonitoring_Request as Request
    from ros2_benchmark_interfaces.srv._start_monitoring import StartMonitoring_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
