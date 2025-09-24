# translate_message.py
# version: 1.0.0
# Original Author: Theodore Tasman
# Creation Date: 2025-09-24
# Last Modified: 2025-09-24
# Organization: PSU UAS

def translate_message(csvm):
    """
    Convert a CSVMessage object to Python dictionary.
    
    Args:
        csvm (CSVMessage): The CSVMessage object to convert.
        
    Returns:
        dict: A dictionary representation of the CSVMessage.
    """
    if csvm.get_type().startswith('UNKNOWN'):
        return {}
    
    fields = csvm.get_fieldnames()

    dict_message = {field: getattr(csvm, field) for field in fields}

    return dict_message