# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros2_benchmark_interfaces:srv/SetData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetData_Request(type):
    """Metaclass of message 'SetData_Request'."""

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
                'ros2_benchmark_interfaces.srv.SetData_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_data__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_data__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_data__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_data__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_data__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'PUBLISH_TF_MESSAGES__DEFAULT': False,
            'PUBLISH_TF_STATIC_MESSAGES__DEFAULT': False,
        }

    @property
    def PUBLISH_TF_MESSAGES__DEFAULT(cls):
        """Return default value for message field 'publish_tf_messages'."""
        return False

    @property
    def PUBLISH_TF_STATIC_MESSAGES__DEFAULT(cls):
        """Return default value for message field 'publish_tf_static_messages'."""
        return False


class SetData_Request(metaclass=Metaclass_SetData_Request):
    """Message class 'SetData_Request'."""

    __slots__ = [
        '_data_path',
        '_publish_tf_messages',
        '_publish_tf_static_messages',
    ]

    _fields_and_field_types = {
        'data_path': 'string',
        'publish_tf_messages': 'boolean',
        'publish_tf_static_messages': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.data_path = kwargs.get('data_path', str())
        self.publish_tf_messages = kwargs.get(
            'publish_tf_messages', SetData_Request.PUBLISH_TF_MESSAGES__DEFAULT)
        self.publish_tf_static_messages = kwargs.get(
            'publish_tf_static_messages', SetData_Request.PUBLISH_TF_STATIC_MESSAGES__DEFAULT)

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
        if self.data_path != other.data_path:
            return False
        if self.publish_tf_messages != other.publish_tf_messages:
            return False
        if self.publish_tf_static_messages != other.publish_tf_static_messages:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def data_path(self):
        """Message field 'data_path'."""
        return self._data_path

    @data_path.setter
    def data_path(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'data_path' field must be of type 'str'"
        self._data_path = value

    @builtins.property
    def publish_tf_messages(self):
        """Message field 'publish_tf_messages'."""
        return self._publish_tf_messages

    @publish_tf_messages.setter
    def publish_tf_messages(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'publish_tf_messages' field must be of type 'bool'"
        self._publish_tf_messages = value

    @builtins.property
    def publish_tf_static_messages(self):
        """Message field 'publish_tf_static_messages'."""
        return self._publish_tf_static_messages

    @publish_tf_static_messages.setter
    def publish_tf_static_messages(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'publish_tf_static_messages' field must be of type 'bool'"
        self._publish_tf_static_messages = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetData_Response(type):
    """Metaclass of message 'SetData_Response'."""

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
                'ros2_benchmark_interfaces.srv.SetData_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_data__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_data__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_data__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_data__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_data__response

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


class SetData_Response(metaclass=Metaclass_SetData_Response):
    """Message class 'SetData_Response'."""

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
        self.success = kwargs.get(
            'success', SetData_Response.SUCCESS__DEFAULT)

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


class Metaclass_SetData(type):
    """Metaclass of service 'SetData'."""

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
                'ros2_benchmark_interfaces.srv.SetData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_data

            from ros2_benchmark_interfaces.srv import _set_data
            if _set_data.Metaclass_SetData_Request._TYPE_SUPPORT is None:
                _set_data.Metaclass_SetData_Request.__import_type_support__()
            if _set_data.Metaclass_SetData_Response._TYPE_SUPPORT is None:
                _set_data.Metaclass_SetData_Response.__import_type_support__()


class SetData(metaclass=Metaclass_SetData):
    from ros2_benchmark_interfaces.srv._set_data import SetData_Request as Request
    from ros2_benchmark_interfaces.srv._set_data import SetData_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
