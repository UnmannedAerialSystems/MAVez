'''
Coordinate class

Ted Tasman
2025-01-25

This class represents a coordinate in latitude, longitude, and altitude.
It provides methods to initialize the coordinate, convert between degrees and decimal degrees,
and offset the coordinate by a given distance and heading.
'''

import math

class Coordinate:

    def __init__(self, lat, lon, alt, dms=False, use_int=True):
        '''
            Initialize the coordinate.
            lat: float | str if dms is True
            lon: float | str if dms is True
            alt: float
            dms: bool - if set to True, the coordinates are in degrees, minutes, seconds
            integer: bool - if set to True, the coordinates are stored as integers
        '''

        # if dms is True, convert the coordinates to decimal degrees
        if dms:
            self.lat = self.dms_to_dd(lat)
            self.lon = self.dms_to_dd(lon)
        # otherwise, use the coordinates as they are
        else:
            self.lat = lat
            self.lon = lon

        # alt is always used as it is
        self.alt = alt

        # if integer is True, convert the coordinates to integers
        if use_int:
            self.lat = int(self.lat * 1e7)
            self.lon = int(self.lon * 1e7)
            self.alt = int(self.alt)
    
    def __str__(self):
        return f'{self.lat} {self.lon} {self.alt}'
    
    __repr__ = __str__

    def dms_to_dd(self, dms):
        '''
            Convert degrees, minutes, seconds to decimal degrees.
            dms: float
            returns:
                dd: float
        '''
        dd = dms[0] + dms[1] / 60 + dms[2] / 3600
        return dd
    

    def offset_coordinate(self, offset, heading):
        '''
        offset: the distance to offset the coordinate in meters
        heading: the direction to offset the coordinate in degrees
        returns:
            Coordinate
        '''

        # convert heading to radians
        heading = math.radians(heading)

        # calculate the offset in latitude and longitude
        lat_offset = offset * math.cos(heading)
        lon_offset = offset * math.sin(heading)

        # convert the offset to degrees
        lat_offset = (lat_offset / 111320) * 10e6
        lon_offset = (lon_offset / (111320 * math.cos(self.lat / 10e6))) * 10e6

        # calculate the new latitude and longitude
        new_lat = self.lat + lat_offset
        new_lon = self.lon + lon_offset

        return Coordinate(int(new_lat), int(new_lon), self.alt, use_int=False)