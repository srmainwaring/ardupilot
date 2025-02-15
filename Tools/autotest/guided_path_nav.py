"""
Generate paths for guided path navigation

Python implementation of a subset of trajectory /  path generation functions
from the ETHZ terrain-navigation package.
https://github.com/ethz-asl/terrain-navigation.git
"""

import math
import pytest
from typing import Self

# Use spatial objects defined in pymavlink
from pymavlink.quaternion import Quaternion
from pymavlink.rotmat import Vector3


# TODO: use versions of wrap_pi, wrap_2pi from autotest suite if available.
def wrap_2pi(angle: float) -> float:
    while (angle < 0.0) or (angle > 2 * math.pi):
        if angle < 0.0:
            angle += 2 * math.pi
        else:
            angle -= 2 * math.pi

    return angle


def wrap_pi(angle: float) -> float:
    while math.fabs(angle) > math.pi:
        if angle > 0:
            angle = angle - 2 * math.pi
        else:
            angle = angle + 2 * math.pi

    return angle


# TODO: move to a self-contained library


class State:
    """
    A kinematic state (position, velocity, orientation)
    """

    def __init__(self):
        self._position = Vector3
        self._velocity = Vector3
        self._attitude = Quaternion

    @property
    def position(self) -> Vector3:
        """
        Return the position.
        """
        return self._position

    @position.setter
    def position(self, value: Vector3) -> None:
        """
        Set the position.
        """
        self._position = value

    @property
    def velocity(self) -> Vector3:
        """
        Return the velocity.
        """
        return self._velocity

    @velocity.setter
    def velocity(self, value: Vector3) -> None:
        """
        Set the velocity.
        """
        self._velocity = value

    @property
    def attitude(self) -> Vector3:
        """
        Return the orientation.
        """
        return self._attitude

    @attitude.setter
    def attitude(self, value: Vector3) -> None:
        """
        Set the orientation.
        """
        self._attitude = value


class PathSegment:
    """
    A path segment comprising a vector of (kinematic) states.
    """

    def __init__(self):
        self._states = []
        self._curvature = 0.0
        self._flightpath_angle = 0.0
        self._dt = 0.0
        # self._utility = 0.0
        # self._viewed = False
        self._reached = False
        self._is_periodic = False

    @property
    def reached(self) -> bool:
        return self._reached

    @reached.setter
    def reached(self, value: bool) -> None:
        self._reached = value

    def first_state(self) -> State:
        """
        Return the first state.
        """
        return self._states[0]

    def last_state(self) -> State:
        """
        Return the last state.
        """
        return self._states[-1]

    def position(self) -> list[Vector3]:
        """
        Return all positions in the path segment.
        """
        pos = [x.position for x in self.states]
        return pos

    def velocity(self) -> list[Vector3]:
        """
        Return all velocities in the path segment.
        """
        vel = [x.velocity for x in self.states]
        return vel

    def attitude(self) -> list[Vector3]:
        """
        Return all orientations in the path segment.
        """
        att = [x.attitude for x in self.states]
        return att

    # TODO: check tangent is a unit vector
    @staticmethod
    def get_arc_centre(segment_start: Vector3, segment_start_tangent: Vector3, curvature: float) -> Vector3:
        """
        Get the centre of an arc given a curvature, segment start and tangent.

        :param Vector3 segment_start: must have a zero z-component
        :param Vector3 segment_start_tangent: must have a zero z-component
        :param float curvature: signed curvature (curvature = kappa = 1 / radius)
        :return: the coordinate vector of the arc centre
        :rtype: Vector3
        :raises ValueError: if the segment_start has non-zero z-componen
        :raises ValueError: if the segment_start_tangent has non-zero z-component
        """
        # verify that z-components are zero
        if not math.isclose(segment_start.z, 0.0):
            raise ValueError("segment_start must have z-component zero")

        if not math.isclose(segment_start_tangent.z, 0.0):
            raise ValueError("segment_start_tangent must have z-component zero")

        # circle radius is inverse of curvature
        kappa = math.fabs(curvature)
        radius = 1.0 / kappa
        direction = curvature / kappa

        # rotation vector - normal to the plane
        rot_vector = Vector3(0.0, 0.0, direction)

        # rotational_vector x segment_start_tangent is a unit vector in the plane
        arc_centre = segment_start + radius * rot_vector % segment_start_tangent
        return arc_centre

    @staticmethod
    def get_arc_centre2(
        segment_start: Vector3, segment_start_tangent: Vector3, curvature: float, segment_end: Vector3
    ) -> Vector3:
        """
        Get the centre of an arc given a curvature, segment start, end and tangent.

        This version uses the segment start and end points to locate the centre,
        with the tangent only being used to determine the orientation.
        """
        # verify that z-components are zero
        if not math.isclose(segment_start.z, 0.0):
            raise ValueError("segment_start must have z-component zero")

        if not math.isclose(segment_start_tangent.z, 0.0):
            raise ValueError("segment_start_tangent must have z-component zero")

        # circle radius is inverse of curvature
        kappa = math.fabs(curvature)
        radius = 1.0 / kappa
        direction = curvature / kappa

        # rotation vector - normal to the plane
        rot_vector = Vector3(0.0, 0.0, direction)

        error_vector = segment_end - segment_start
        segment_distance = error_vector.length()
        center_distance = math.sqrt(math.pow(radius, 2) - math.pow(0.5 * segment_distance, 2))

        midpoint_2d = 0.5 * (segment_start + segment_end)
        distance_vector = error_vector.normalized()
        normal_vector = distance_vector % rot_vector
        if error_vector * segment_start_tangent > 0.0:
            normal_vector = -1.0 * distance_vector % rot_vector

        arc_center = midpoint_2d + center_distance * normal_vector
        return arc_center

    # TODO: check segment_start does not equal segment_end
    @staticmethod
    def get_line_progress(position: Vector3, segment_start: Vector3, segment_end: Vector3) -> float:
        """
        Calculate the fraction of the linear distance between the segment
        start and end.
        """
        resultant_vector = segment_end - segment_start
        direction_vector = resultant_vector.normalized()
        segment_length = resultant_vector.length()
        error_vector = position - segment_start
        theta = (error_vector * direction_vector) / segment_length
        return theta

    @staticmethod
    def get_arc_progress(
        arc_centre: Vector3, position: Vector3, segment_start: Vector3, segment_end: Vector3, curvature: float
    ) -> float:
        """
        Calculate the fraction of the arc distance between the segment
        start and end.
        """
        # verify that z-components are zero
        if not math.isclose(arc_centre.z, 0.0):
            raise ValueError("arc_centre must have z-component zero")

        if not math.isclose(position.z, 0.0):
            raise ValueError("position must have z-component zero")

        if not math.isclose(segment_start.z, 0.0):
            raise ValueError("segment_start must have z-component zero")

        if not math.isclose(segment_end.z, 0.0):
            raise ValueError("segment_end must have z-component zero")

        # unit vectors from arc-centre to positions on the arc
        pos_vector = (position - arc_centre).normalized()
        start_vector = (segment_start - arc_centre).normalized()
        end_vector = (segment_end - arc_centre).normalized()

        # angles between vectors from arc-centre to positions on the arc
        psi = math.atan2(end_vector.y, end_vector.x) - math.atan2(start_vector.y, start_vector.x)
        angle_pos = math.atan2(pos_vector.y, pos_vector.x) - math.atan2(start_vector.y, start_vector.x)
        if curvature < 0.0:
            psi *= -1.0
            angle_pos *= -1.0

        wrap_2pi(psi)
        wrap_2pi(angle_pos)

        residual_pi = math.max(2.0 * math.pi - psi, 0.0)

        if angle_pos > psi + 0.5 * residual_pi and residual_pi > 0.0:
            angle_pos = angle_pos - 2 * math.pi

        theta = angle_pos / psi
        return theta

    def get_length(self, epsilon: float = 1.0e-3) -> float:
        if len(self._states) == 0:
            raise RuntimeError("PathSegment is empty")

        if len(self._states) == 1:
            return 0.0

        segment_start = self._states[0].position
        segment_end = self._states[0].position
        if math.fabs(self._curvature) < 0.0001:
            # segment is a line segment
            length = (segment_end - segment_start).length()
            return length

        # compute closest point on a arc segment
        segment_start_2d = Vector3(segment_start.x, segment_start.y, 0.0)
        segment_end_2d = Vector3(segment_end.x, segment_end.y, 0.0)
        if (segment_start_2d - segment_end_2d).length() < epsilon:
            # return full circle length
            length = 2 * math.pi * (1.0 / math.fabs(self._curvature))
        else:
            segment_start_2d = Vector3(segment_start.x, segment_start.y, 0.0)
            segment_end_2d = Vector3(segment_end.x, segment_end.y, 0.0)
            segment_velocity = self._states[0].velocity
            segment_start_tangent_2d = Vector3(segment_velocity.x, segment_velocity.y, 0.0).normalized()

            arc_center_2d = PathSegment.get_arc_centre(segment_start_2d, segment_start_tangent_2d, self._curvature)
            start_vector = (segment_start_2d - arc_center_2d).normalized()
            end_vector = (segment_end_2d - arc_center_2d).normalized()

            psi = math.atan2(end_vector.y, end_vector.x) - math.atan2(start_vector.y, start_vector.x)
            wrap_2pi(psi)
            length = (1.0 / math.fabs(self._curvature)) * psi

        return length

    def get_closest_point(self, position: Vector3, epsilon: float = 1.0e-4) -> tuple[float, Vector3, Vector3:float]:
        theta = -1.0 * math.inf
        closest_point = Vector3()
        tangent = Vector3()

        if len(self._states) == 0:
            raise RuntimeError("PathSegment is empty")

        segment_start = self._states[0].position
        segment_start_tangent = (self._states[0].velocity).normalized()
        segment_end = self._states[-1].position

        if len(self._states) == 1:
            # segment only contains a single state, meaning that it is not a line or arc
            theta = 1.0
        elif math.abs(self._curvature) < epsilon:
            # compute closest point on a line segment
            theta = PathSegment.get_line_progress(position, segment_start, segment_end)
            tangent = (segment_end - segment_start).normalized()
            # closest point should not be outside segment - constrain theta to [0.0, 1.0]
            closest_point = math.max(math.min(1.0, theta), 0.0) * (segment_end - segment_start) + segment_start
        else:
            # compute closest point on a arc segment
            position_2d = Vector3(position.x, position.y, 0.0)
            segment_start_2d = Vector3(segment_start.x, segment_start.y, 0.0)
            segment_start_tangent_2d = Vector3(segment_start_tangent.x, segment_start_tangent.y, 0.0).normalized()
            segment_end_2d = Vector3(segment_end.x, segment_end.y, 0.0)
            arc_center = Vector3()
            # handle when it is a full circle
            if self._is_periodic:
                arc_center = PathSegment.get_arc_centre(segment_start_2d, segment_start_tangent_2d, self._curvature)
                start_vector = (segment_start_2d - arc_center).normalized()
                position_vector = position_2d - arc_center
                angle_pos = math.atan2(position_vector.y, position_vector.x) - math.atan2(
                    start_vector.y, start_vector.x
                )
                wrap_2pi(angle_pos)
                # TODO: Check for the case for a helix!
                theta = angle_pos / (2 * math.pi)
            else:
                arc_center = PathSegment.get_arc_centre2(
                    self._curvature, segment_start_2d, segment_start_tangent_2d, segment_end_2d
                )
                theta = PathSegment.get_arc_progress(
                    arc_center, position_2d, segment_start_2d, segment_end_2d, self._curvature
                )

            closest_point_2d = math.fabs(1.0 / self._curvature) * (position_2d - arc_center).normalized() + arc_center
            closest_point = Vector3(
                closest_point_2d.x, closest_point_2d.y, theta * segment_end.z + (1.0 - theta) * segment_start.z
            )
            # position to error vector
            error_vector = (closest_point_2d - arc_center).normalized()
            tangent = Vector3(
                (self._curvature / math.fabs(self._curvature)) * error_vector.y * -1.0,
                (self._curvature / math.fabs(self._curvature)) * error_vector.x,
                0.0,
            )

        tangent = Vector3(
            math.cos(self._flightpath_angle) * tangent.x,
            math.cos(self._flightpath_angle) * tangent.y,
            math.sin(self._flightpath_angle),
        )

        return (theta, closest_point, tangent, self._curvature)


class Path:
    """
    A path comprising a vector of path segments.
    """

    def __init__(self):
        self._segments = []
        self._utility = 0.0
        self._is_valid = False
        self._epsilon = 15.0 * 0.2

    @property
    def is_valid(self) -> bool:
        return self._is_valid

    def position(self) -> list[Vector3]:
        result = []
        for seg in self._segments:
            pos = seg.position()
            result.extend(pos)
        return result

    def velocity(self) -> list[Vector3]:
        result = []
        for seg in self._segments:
            vel = seg.velocity()
            result.extend(vel)
        return result

    def attitude(self) -> list[Vector3]:
        result = []
        for seg in self._segments:
            att = seg.attitude()
            result.extend(att)
        return result

    def reset_segments(self) -> None:
        self._segments.clear()

    def prepend_segment(self, path_segment: PathSegment) -> None:
        self._segments.insert(0, path_segment)

    def append_segment(self, path_segment: PathSegment) -> None:
        self._segments.append(path_segment)

    def append_path(self, path: Self) -> None:
        for seg in path._segments:
            self.append_segment(seg)

    def first_segment(self) -> PathSegment:
        return self._segments[0]

    def last_segment(self) -> PathSegment:
        return self._segments[-1]

    def get_closest_point(self, position: Vector3) -> tuple[Vector3, Vector3, float]:

        closest_point = Vector3()
        tangent = Vector3()
        curvature = 0.0

        # TODO: accessing private member - add method
        closest_point = self._segments[0].first_state().position

        # iterate through all segments
        for segment in self._segments:
            if segment.reached and (segment is not self._segments[-1]):
                continue
            (theta, closest_point, tangent, curvature) = segment.get_closest_point(position)

            curvature = segment._curvature
            if theta <= 0.0:
                # theta can be negative on the start of the next segment
                closest_point = segment.first_state().position
                tangent = (segment.first_state().velocity).normalized()
                return (closest_point, tangent, curvature)
            elif theta < 1.0:
                # TODO: This is a magic number in terms of acceptance radius of end of segments
                return (closest_point, tangent, curvature)
            else:
                closest_point = segment.states.back().position
                tangent = (segment.states.back().velocity).normalized()
                # consider that the segment is tracked
                segment.reached = True

        # TODO: Handle the case when all segments are marked as reached
        return (closest_point, tangent, curvature)

    def get_end_of_current_segment(self, position: Vector3) -> Vector3:
        end_of_current_segment = Vector3()
        return end_of_current_segment

    def get_current_segment(self, position: Vector3) -> PathSegment:
        current_segment = PathSegment()
        return current_segment

    def get_current_segment_index(self, position: Vector3) -> int:
        current_segment_index = 0
        return current_segment_index

    def get_length(self, start_index: int = 0) -> float:
        length = 0.0
        return length


# Test cases (pytest)
def test_arc_centre():
    segment_start = Vector3(10.0, 10.0, 0.0)
    segment_start_tangent = Vector3(1.0, -1.0, 0.0) / math.sqrt(2.0)
    radius = math.sqrt(2.0) * 10.0
    curvature = 1.0 / radius

    assert segment_start.x == 10.0
    assert segment_start.y == 10.0
    assert segment_start_tangent.x == 1.0 / math.sqrt(2.0)
    assert segment_start_tangent.y == -1.0 / math.sqrt(2.0)

    arc_centre = PathSegment.get_arc_centre(segment_start, segment_start_tangent, curvature)
    assert arc_centre.x == 20.0
    assert arc_centre.y == 20.0

    arc_centre = PathSegment.get_arc_centre(segment_start, segment_start_tangent, -1.0 * curvature)
    assert arc_centre.x == 0.0
    assert arc_centre.y == 0.0


def test_arc_centre2():
    segment_start = Vector3(10.0, 10.0, 0.0)
    segment_end = Vector3(20.0, 20.0 - 10.0 * math.sqrt(2.0), 0.0)
    segment_start_tangent = Vector3(1.0, -1.0, 0.0) / math.sqrt(2.0)
    radius = math.sqrt(2.0) * 10.0
    curvature = 1.0 / radius

    assert segment_start.x == 10.0
    assert segment_start.y == 10.0
    assert segment_end.x == 20.0
    assert segment_end.y == 20.0 - 10.0 * math.sqrt(2.0)
    assert segment_start_tangent.x == 1.0 / math.sqrt(2.0)
    assert segment_start_tangent.y == -1.0 / math.sqrt(2.0)

    arc_centre = PathSegment.get_arc_centre2(segment_start, segment_start_tangent, curvature, segment_end)
    assert arc_centre.x == 20.0
    assert arc_centre.y == 20.0


def test_line_progress():
    segment_start = Vector3(10.0, 10.0, 0.0)
    segment_end = Vector3(20.0, 20.0, 0.0)

    position = Vector3(11.0, 11.0, 0.0)
    theta = PathSegment.get_line_progress(position, segment_start, segment_end)
    assert theta == pytest.approx(0.1)

    position = Vector3(15.0, 15.0, 0.0)
    theta = PathSegment.get_line_progress(position, segment_start, segment_end)
    assert theta == pytest.approx(0.5)

    position = Vector3(18.0, 18.0, 0.0)
    theta = PathSegment.get_line_progress(position, segment_start, segment_end)
    assert theta == pytest.approx(0.8)
