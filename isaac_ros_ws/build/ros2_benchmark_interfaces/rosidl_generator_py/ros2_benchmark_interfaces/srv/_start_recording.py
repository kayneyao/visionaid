# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros2_benchmark_interfaces:srv/StartRecording.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_StartRecording_Request(type):
    """Metaclass of message 'StartRecording_Request'."""

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
                'ros2_benchmark_interfaces.srv.StartRecording_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__start_recording__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__start_recording__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__start_recording__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__start_recording__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__start_recording__request

            from ros2_benchmark_interfaces.msg import TopicMessageTimestampArray
            if TopicMessageTimestampArray.__class__._TYPE_SUPPORT is None:
                TopicMessageTimestampArray.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'RECORD_DATA_TIMELINE__DEFAULT': False,
        }

    @property
    def RECORD_DATA_TIMELINE__DEFAULT(cls):
        """Return default value for message field 'record_data_timeline'."""
        return False


class StartRecording_Request(metaclass=Metaclass_StartRecording_Request):
    """Message class 'StartRecording_Request'."""

    __slots__ = [
        '_buffer_length',
        '_timeout',
        '_topic_message_timestamps',
        '_record_data_timeline',
    ]

    _fields_and_field_types = {
        'buffer_length': 'uint64',
        'timeout': 'int64',
        'topic_message_timestamps': 'sequence<ros2_benchmark_interfaces/TopicMessageTimestampArray>',
        'record_data_timeline': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['ros2_benchmark_interfaces', 'msg'], 'TopicMessageTimestampArray')),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.buffer_length = kwargs.get('buffer_length', int())
        self.timeout = kwargs.get('timeout', int())
        self.topic_message_timestamps = kwargs.get('topic_message_timestamps', [])
        self.record_data_timeline = kwargs.get(
            'record_data_timeline', StartRecording_Request.RECORD_DATA_TIMELINE__DEFAULT)

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
        if self.buffer_length != other.buffer_length:
            return False
        if self.timeout != other.timeout:
            return False
        if self.topic_message_timestamps != other.topic_message_timestamps:
            return False
        if self.record_data_timeline != other.record_data_timeline:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def buffer_length(self):
        """Message field 'buffer_length'."""
        return self._buffer_length

    @buffer_length.setter
    def buffer_length(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'buffer_length' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'buffer_length' field must be an unsigned integer in [0, 18446744073709551615]"
        self._buffer_length = value

    @builtins.property
    def timeout(self):
        """Message field 'timeout'."""
        return self._timeout

    @timeout.setter
    def timeout(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timeout' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'timeout' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._timeout = value

    @builtins.property
    def topic_message_timestamps(self):
        """Message field 'topic_message_timestamps'."""
        return self._topic_message_timestamps

    @topic_message_timestamps.setter
    def topic_message_timestamps(self, value):
        if __debug__:
            from ros2_benchmark_interfaces.msg import TopicMessageTimestampArray
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, TopicMessageTimestampArray) for v in value) and
                 True), \
                "The 'topic_message_timestamps' field must be a set or sequence and each value of type 'TopicMessageTimestampArray'"
        self._topic_message_timestamps = value

    @builtins.property
    def record_data_timeline(self):
        """Message field 'record_data_timeline'."""
        return self._record_data_timeline

    @record_data_timeline.setter
    def record_data_timeline(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'record_data_timeline' field must be of type 'bool'"
        self._record_data_timeline = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_StartRecording_Response(type):
    """Metaclass of message 'StartRecording_Response'."""

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
                'ros2_benchmark_interfaces.srv.StartRecording_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__start_recording__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__start_recording__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__start_recording__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__start_recording__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__start_recording__response

            from ros2_benchmark_interfaces.msg import TopicMessageCount
            if TopicMessageCount.__class__._TYPE_SUPPORT is None:
                TopicMessageCount.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'SUCCESS__DEFAULT': False,
        }

    @property
    def SUCCESS__DEFAULT(cls):
        """Return default value for message field 'success'."""
        return False


class StartRecording_Response(metaclass=Metaclass_StartRecording_Response):
    """Message class 'StartRecording_Response'."""

    __slots__ = [
        '_success',
        '_recorded_message_count',
        '_recorded_topic_message_counts',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'recorded_message_count': 'uint64',
        'recorded_topic_message_counts': 'sequence<ros2_benchmark_interfaces/TopicMessageCount>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['ros2_benchmark_interfaces', 'msg'], 'TopicMessageCount')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get(
            'success', StartRecording_Response.SUCCESS__DEFAULT)
        self.recorded_message_count = kwargs.get('recorded_message_count', int())
        self.recorded_topic_message_counts = kwargs.get('recorded_topic_message_counts', [])

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
        if self.recorded_message_count != other.recorded_message_count:
            return False
        if self.recorded_topic_message_counts != other.recorded_topic_message_counts:
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

    @builtins.property
    def recorded_message_count(self):
        """Message field 'recorded_message_count'."""
        return self._recorded_message_count

    @recorded_message_count.setter
    def recorded_message_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'recorded_message_count' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'recorded_message_count' field must be an unsigned integer in [0, 18446744073709551615]"
        self._recorded_message_count = value

    @builtins.property
    def recorded_topic_message_counts(self):
        """Message field 'recorded_topic_message_counts'."""
        return self._recorded_topic_message_counts

    @recorded_topic_message_counts.setter
    def recorded_topic_message_counts(self, value):
        if __debug__:
            from ros2_benchmark_interfaces.msg import TopicMessageCount
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, TopicMessageCount) for v in value) and
                 True), \
                "The 'recorded_topic_message_counts' field must be a set or sequence and each value of type 'TopicMessageCount'"
        self._recorded_topic_message_counts = value


class Metaclass_StartRecording(type):
    """Metaclass of service 'StartRecording'."""

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
                'ros2_benchmark_interfaces.srv.StartRecording')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__start_recording

            from ros2_benchmark_interfaces.srv import _start_recording
            if _start_recording.Metaclass_StartRecording_Request._TYPE_SUPPORT is None:
                _start_recording.Metaclass_StartRecording_Request.__import_type_support__()
            if _start_recording.Metaclass_StartRecording_Response._TYPE_SUPPORT is None:
                _start_recording.Metaclass_StartRecording_Response.__import_type_support__()


class StartRecording(metaclass=Metaclass_StartRecording):
    from ros2_benchmark_interfaces.srv._start_recording import StartRecording_Request as Request
    from ros2_benchmark_interfaces.srv._start_recording import StartRecording_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
