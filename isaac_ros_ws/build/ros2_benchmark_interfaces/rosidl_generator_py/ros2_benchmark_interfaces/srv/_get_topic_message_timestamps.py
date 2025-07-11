# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros2_benchmark_interfaces:srv/GetTopicMessageTimestamps.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetTopicMessageTimestamps_Request(type):
    """Metaclass of message 'GetTopicMessageTimestamps_Request'."""

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
                'ros2_benchmark_interfaces.srv.GetTopicMessageTimestamps_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_topic_message_timestamps__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_topic_message_timestamps__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_topic_message_timestamps__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_topic_message_timestamps__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_topic_message_timestamps__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'START_TIME_OFFSET_NS__DEFAULT': -1,
            'END_TIME_OFFSET_NS__DEFAULT': -1,
        }

    @property
    def START_TIME_OFFSET_NS__DEFAULT(cls):
        """Return default value for message field 'start_time_offset_ns'."""
        return -1

    @property
    def END_TIME_OFFSET_NS__DEFAULT(cls):
        """Return default value for message field 'end_time_offset_ns'."""
        return -1


class GetTopicMessageTimestamps_Request(metaclass=Metaclass_GetTopicMessageTimestamps_Request):
    """Message class 'GetTopicMessageTimestamps_Request'."""

    __slots__ = [
        '_start_time_offset_ns',
        '_end_time_offset_ns',
    ]

    _fields_and_field_types = {
        'start_time_offset_ns': 'int64',
        'end_time_offset_ns': 'int64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.start_time_offset_ns = kwargs.get(
            'start_time_offset_ns', GetTopicMessageTimestamps_Request.START_TIME_OFFSET_NS__DEFAULT)
        self.end_time_offset_ns = kwargs.get(
            'end_time_offset_ns', GetTopicMessageTimestamps_Request.END_TIME_OFFSET_NS__DEFAULT)

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
        if self.start_time_offset_ns != other.start_time_offset_ns:
            return False
        if self.end_time_offset_ns != other.end_time_offset_ns:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def start_time_offset_ns(self):
        """Message field 'start_time_offset_ns'."""
        return self._start_time_offset_ns

    @start_time_offset_ns.setter
    def start_time_offset_ns(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'start_time_offset_ns' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'start_time_offset_ns' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._start_time_offset_ns = value

    @builtins.property
    def end_time_offset_ns(self):
        """Message field 'end_time_offset_ns'."""
        return self._end_time_offset_ns

    @end_time_offset_ns.setter
    def end_time_offset_ns(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'end_time_offset_ns' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'end_time_offset_ns' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._end_time_offset_ns = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GetTopicMessageTimestamps_Response(type):
    """Metaclass of message 'GetTopicMessageTimestamps_Response'."""

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
                'ros2_benchmark_interfaces.srv.GetTopicMessageTimestamps_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_topic_message_timestamps__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_topic_message_timestamps__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_topic_message_timestamps__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_topic_message_timestamps__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_topic_message_timestamps__response

            from ros2_benchmark_interfaces.msg import TopicMessageTimestampArray
            if TopicMessageTimestampArray.__class__._TYPE_SUPPORT is None:
                TopicMessageTimestampArray.__class__.__import_type_support__()

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


class GetTopicMessageTimestamps_Response(metaclass=Metaclass_GetTopicMessageTimestamps_Response):
    """Message class 'GetTopicMessageTimestamps_Response'."""

    __slots__ = [
        '_topic_message_timestamps',
        '_success',
    ]

    _fields_and_field_types = {
        'topic_message_timestamps': 'sequence<ros2_benchmark_interfaces/TopicMessageTimestampArray>',
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['ros2_benchmark_interfaces', 'msg'], 'TopicMessageTimestampArray')),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.topic_message_timestamps = kwargs.get('topic_message_timestamps', [])
        self.success = kwargs.get(
            'success', GetTopicMessageTimestamps_Response.SUCCESS__DEFAULT)

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
        if self.topic_message_timestamps != other.topic_message_timestamps:
            return False
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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


class Metaclass_GetTopicMessageTimestamps(type):
    """Metaclass of service 'GetTopicMessageTimestamps'."""

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
                'ros2_benchmark_interfaces.srv.GetTopicMessageTimestamps')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_topic_message_timestamps

            from ros2_benchmark_interfaces.srv import _get_topic_message_timestamps
            if _get_topic_message_timestamps.Metaclass_GetTopicMessageTimestamps_Request._TYPE_SUPPORT is None:
                _get_topic_message_timestamps.Metaclass_GetTopicMessageTimestamps_Request.__import_type_support__()
            if _get_topic_message_timestamps.Metaclass_GetTopicMessageTimestamps_Response._TYPE_SUPPORT is None:
                _get_topic_message_timestamps.Metaclass_GetTopicMessageTimestamps_Response.__import_type_support__()


class GetTopicMessageTimestamps(metaclass=Metaclass_GetTopicMessageTimestamps):
    from ros2_benchmark_interfaces.srv._get_topic_message_timestamps import GetTopicMessageTimestamps_Request as Request
    from ros2_benchmark_interfaces.srv._get_topic_message_timestamps import GetTopicMessageTimestamps_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
