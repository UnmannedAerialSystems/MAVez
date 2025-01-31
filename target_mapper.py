'''
target_mapper.py
version 1.0.0

This module is responsible for converting object detection results into target coordinates.
Currently, it generates a random coordinate near the test site.
'''


from flight_utils import Coordinate

class Target_Mapper:

    def __init__(self):
        pass

    def get_target(self):
        '''
            Generate a random target coordinate.
            returns:
                Coordinate object
        '''
        import random
        lat = random.uniform(38.315, 38.316)
        lon = random.uniform(-76.548, -76.549)

        return Coordinate(lat, lon, 0)

