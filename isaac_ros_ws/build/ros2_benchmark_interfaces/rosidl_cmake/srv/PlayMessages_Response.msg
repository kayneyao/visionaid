

# Whether or not playback proceeds successfully. Possible failures include:
# - Message timeline information is not available when using PLAYBACK_MODE_TIMELINE
# - Failed to store a message's publish timestamp due to duplicate message keys
# - Not all expected messages are present in the buffers before starting playback
# - Key duplication in buffered messages provided to the node
# - The target publisher rate cannot be met when enforce_publisher_rate is enabled
bool success false

# The keys and timestamps of the messages that have been published
TimestampedMessageArray timestamps
