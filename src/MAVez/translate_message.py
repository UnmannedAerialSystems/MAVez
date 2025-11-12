# translate_message.py
# version: 2.0.0
# Original Author: Theodore Tasman
# Creation Date: 2025-09-24
# Last Modified: 2025-09-24
# Organization: PSU UAS

from uas_messenger.message import Message

def translate_message(csvm, topic: str = "") -> Message | None:
    """
    Convert a CSVMessage object to Python dictionary.
    
    Args:
        csvm (CSVMessage): The CSVMessage object to convert.
        
    Returns:
        Message: a uas_messenger Message object containing the data from the CSVMessage. If the message type is 'UNKNOWN', returns None.
    """
    if csvm.get_type().startswith('UNKNOWN'):
        return None

    fields = csvm.get_fieldnames()

    data = {field: getattr(csvm, field) for field in fields}

    return Message(topic=f"{topic}_{csvm.get_type()}" if topic else csvm.get_type(), header=data)