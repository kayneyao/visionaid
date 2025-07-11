# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros2_benchmark_interfaces:srv/PlayMessages.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PlayMessages_Request(type):
    """Metaclass of message 'PlayMessages_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'PLAYBACK_MODE_TIMELINE': 0,
        'PLAYBACK_MODE_LOOPING': 1,
        'PLAYBACK_MODE_SWEEPING': 2,
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
                'ros2_benchmark_interfaces.srv.PlayMessages_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__play_messages__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__play_messages__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__play_messages__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__play_messages__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__play_messages__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'PLAYBACK_MODE_TIMELINE': cls.__constants['PLAYBACK_MODE_TIMELINE'],
            'PLAYBACK_MODE_LOOPING': cls.__constants['PLAYBACK_MODE_LOOPING'],
            'PLAYBACK_MODE_SWEEPING': cls.__constants['PLAYBACK_MODE_SWEEPING'],
            'PLAYBACK_MODE__DEFAULT': 1,
            'MESSAGE_COUNT__DEFAULT': 0,
            'ENFORCE_PUBLISHER_RATE__DEFAULT': False,
            'REVISE_TIMESTAMPS_AS_MESSAGE_IDS__DEFAULT': False,
        }

    @property
    def PLAYBACK_MODE_TIMELINE(self):
        """Message constant 'PLAYBACK_MODE_TIMELINE'."""
        return Metaclass_PlayMessages_Request.__constants['PLAYBACK_MODE_TIMELINE']

    @property
    def PLAYBACK_MODE_LOOPING(self):
        """Message constant 'PLAYBACK_MODE_LOOPING'."""
        return Metaclass_PlayMessages_Request.__constants['PLAYBACK_MODE_LOOPING']

    @property
    def PLAYBACK_MODE_SWEEPING(self):
        """Message constant 'PLAYBACK_MODE_SWEEPING'."""
        return Metaclass_PlayMessages_Request.__constants['PLAYBACK_MODE_SWEEPING']

    @property
    def PLAYBACK_MODE__DEFAULT(cls):
        """Return default value for message field 'playback_mode'."""
        return 1

    @property
    def MESSAGE_COUNT__DEFAULT(cls):
        """Return default value for message field 'message_count'."""
        return 0

    @property
    def ENFORCE_PUBLISHER_RATE__DEFAULT(cls):
        """Return default value for message field 'enforce_publisher_rate'."""
        return False

    @property
    def REVISE_TIMESTAMPS_AS_MESSAGE_IDS__DEFAULT(cls):
        """Return default value for message field 'revise_timestamps_as_message_ids'."""
        return False


class PlayMessages_Request(metaclass=Metaclass_PlayMessages_Request):
    """
    Message class 'PlayMessages_Request'.

    Constants:
      PLAYBACK_MODE_TIMELINE
      PLAYBACK_MODE_LOOPING
      PLAYBACK_MODE_SWEEPING
    """

    __slots__ = [
        '_playback_mode',
        '_message_count',
        '_target_publisher_rate',
        '_enforce_publisher_rate',
        '_revise_timestamps_as_message_ids',
    ]

    _fields_and_field_types = {
        'playback_mode': 'uint8',
        'message_count': 'uint64',
        'target_publisher_rate': 'double',
        'enforce_publisher_rate': 'boolean',
        'revise_timestamps_as_message_ids': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.playback_mode = kwargs.get(
            'playback_mode', PlayMessages_Request.PLAYBACK_MODE__DEFAULT)
        self.message_count = kwargs.get(
            'message_count', PlayMessages_Request.MESSAGE_COUNT__DEFAULT)
        self.target_publisher_rate = kwargs.get('target_publisher_rate', float())
        self.enforce_publisher_rate = kwargs.get(
            'enforce_publisher_rate', PlayMessages_Request.ENFORCE_PUBLISHER_RATE__DEFAULT)
        self.revise_timestamps_as_message_ids = kwargs.get(
            'revise_timestamps_as_message_ids', PlayMessages_Request.REVISE_TIMESTAMPS_AS_MESSAGE_IDS__DEFAULT)

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
        if self.playback_mode != other.playback_mode:
            return False
        if self.message_count != other.message_count:
            return False
        if self.target_publisher_rate != other.target_publisher_rate:
            return False
        if self.enforce_publisher_rate != other.enforce_publisher_rate:
            return False
        if self.revise_timestamps_as_message_ids != other.revise_timestamps_as_message_ids:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def playback_mode(self):
        """Message field 'playback_mode'."""
        return self._playback_mode

    @playback_mode.setter
    def playback_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'playback_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'playback_mode' field must be an unsigned integer in [0, 255]"
        self._playback_mode = value

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
    def target_publisher_rate(self):
        """Message field 'target_publisher_rate'."""
        return self._target_publisher_rate

    @target_publisher_rate.setter
    def target_publisher_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_publisher_rate' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'target_publisher_rate' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._target_publisher_rate = value

    @builtins.property
    def enforce_publisher_rate(self):
        """Message field 'enforce_publisher_rate'."""
        return self._enforce_publisher_rate

    @enforce_publisher_rate.setter
    def enforce_publisher_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'enforce_publisher_rate' field must be of type 'bool'"
        self._enforce_publisher_rate = value

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


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PlayMessages_Response(type):
    """Metaclass of message 'PlayMessages_Response'."""

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
                'ros2_benchmark_interfaces.srv.PlayMessages_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__play_messages__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__play_messages__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__play_messages__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__play_messages__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__play_messages__response

            from ros2_benchmark_interfaces.msg import TimestampedMessageArray
            if TimestampedMessageArray.__class__._TYPE_SUPPORT is None:
                TimestampedMessageArray.__class__.__import_type_support__()

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


class PlayMessages_Response(metaclass=Metaclass_PlayMessages_Response):
    """Message class 'PlayMessages_Response'."""

    __slots__ = [
        '_success',
        '_timestamps',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'timestamps': 'ros2_benchmark_interfaces/TimestampedMessageArray',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ros2_benchmark_interfaces', 'msg'], 'TimestampedMessageArray'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get(
            'success', PlayMessages_Response.SUCCESS__DEFAULT)
        from ros2_benchmark_interfaces.msg import TimestampedMessageArray
        self.timestamps = kwargs.get('timestamps', TimestampedMessageArray())

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
        if self.timestamps != other.timestamps:
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
    def timestamps(self):
        """Message field 'timestamps'."""
        return self._timestamps

    @timestamps.setter
    def timestamps(self, value):
        if __debug__:
            from ros2_benchmark_interfaces.msg import TimestampedMessageArray
            assert \
                isinstance(value, TimestampedMessageArray), \
                "The 'timestamps' field must be a sub message of type 'TimestampedMessageArray'"
        self._timestamps = value


class Metaclass_PlayMessages(type):
    """Metaclass of service 'PlayMessages'."""

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
                'ros2_benchmark_interfaces.srv.PlayMessages')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__play_messages

            from ros2_benchmark_interfaces.srv import _play_messages
            if _play_messages.Metaclass_PlayMessages_Request._TYPE_SUPPORT is None:
                _play_messages.Metaclass_PlayMessages_Request.__import_type_support__()
            if _play_messages.Metaclass_PlayMessages_Response._TYPE_SUPPORT is None:
                _play_messages.Metaclass_PlayMessages_Response.__import_type_support__()


class PlayMessages(metaclass=Metaclass_PlayMessages):
    from ros2_benchmark_interfaces.srv._play_messages import PlayMessages_Request as Request
    from ros2_benchmark_interfaces.srv._play_messages import PlayMessages_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
