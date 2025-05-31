'''
Coordinate class

Ted Tasman
2025-01-25

This class represents a coordinate in latitude, longitude, and altitude.
It provides methods to initialize the coordinate, convert between degrees and decimal degrees,
and offset the coordinate by a given distance and heading.
'''

import math

EARTH_RADIUS = 6378137  # in meters

class Coordinate:

    def __init__(self, lat, lon, alt, dms=False, use_int=True, heading=None):
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
        
        self.is_int = use_int

        self.heading = heading
    
    def __str__(self):
        return f'{self.lat},{self.lon},{self.alt},{self.heading}' if self.heading is not None else f'{self.lat},{self.lon},{self.alt}'
    
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

        lat, lon = self.normalize()

        # convert heading to radians
        heading = math.radians(heading)

        # calculate the offset in latitude and longitude
        lat_offset = offset * math.cos(heading)
        lon_offset = offset * math.sin(heading)

        # convert the offset to degrees
        lat_offset = (lat_offset / 111320)
        lon_offset = (lon_offset / (111320 * math.cos(lat / 10e6)))

        # calculate the new latitude and longitude
        new_lat = lat + lat_offset
        new_lon = lon + lon_offset

        return Coordinate(new_lat, new_lon, self.alt, use_int=self.is_int)
    

    def __eq__(self, other):
        '''
            Compare two coordinates.
            returns:
                True if the coordinates are equal
                False if the coordinates are not equal
        '''
        if isinstance(other, tuple) and len(other) == 3:
            # if other is a tuple, convert it to a Coordinate
            other = Coordinate(other[0], other[1], other[2])

        return self.lat == other.lat and self.lon == other.lon and self.alt == other.alt
    

    def normalize(self):
        '''
            Normalize the coordinates to decimal degrees.
            returns:
                tuple of (lat, lon) in radians
        '''
        # ensure both self and other are in decimal degrees
        if self.is_int:
            self_lat = self.lat / 1e7
            self_lon = self.lon / 1e7
        else:
            self_lat = self.lat
            self_lon = self.lon
        
        return self_lat, self_lon

    
    def distance_to(self, other):
        '''
            Calculate the distance between two coordinates in meters using the haversine formula.
            other: Coordinate
            returns:
                distance: float
        '''
        # ensure other is a Coordinate
        if not isinstance(other, Coordinate):
            raise TypeError('other must be a Coordinate')
        
        self_lat, self_lon = self.normalize()
        other_lat, other_lon = other.normalize()

        dlat = math.radians(other_lat - self_lat)
        dlon = math.radians(other_lon - self_lon)

        a = (math.sin(dlat / 2) ** 2 +
             math.cos(math.radians(self_lat)) * math.cos(math.radians(other_lat)) *
             math.sin(dlon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = EARTH_RADIUS * c
        return distance


    def bearing_to(self, other):
        '''
            Calculate the bearing between two coordinates in degrees.
            other: Coordinate
            returns:
                bearing: float 
        '''
        # ensure other is a Coordinate
        if not isinstance(other, Coordinate):
            raise TypeError('other must be a Coordinate')

        self_lat, self_lon = self.normalize()
        other_lat, other_lon = other.normalize()

        dlon = math.radians(other_lon - self_lon)
        self_lat = math.radians(self_lat)
        other_lat = math.radians(other_lat)

        x = math.sin(dlon) * math.cos(other_lat)
        y = (math.cos(self_lat) * math.sin(other_lat) - math.sin(self_lat) * math.cos(other_lat) * math.cos(dlon))
        initial_bearing = math.atan2(x, y)
        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360
        return compass_bearing
    

def main():
    coord1 = Coordinate(37.7749, -122.4194, 0, use_int=True)
    coord2 = Coordinate(34.0522, -118.2437, 0, use_int=True)

    print(f"Distance: {coord1.distance_to(coord2)} meters")
    print(f"Bearing: {coord1.bearing_to(coord2)} degrees")

    offset_coord = coord1.offset_coordinate(1000, 45)
    print(f"Offset Coordinate: {offset_coord}")


if __name__ == "__main__":
    main()