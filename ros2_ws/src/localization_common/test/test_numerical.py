"""Pure numerical geodesy and axis tests do not import ROS."""

import numpy as np
import pytest
from localization_common.geodesy import make_enu_origin, geodetic_to_enu, enu_to_geodetic
from localization_common.frames import (planar_heading_pose, px4_pose, quaternion,
                                        rotation, ugv_pose)


@pytest.mark.parametrize("point", [[0., 0., 0.], [100., -50., 3.], [-5., 8., -2.]])
def test_geodetic_round_trip(point):
    origin = make_enu_origin(47.397971057728974, 8.546163739800146, 0.)
    geo = enu_to_geodetic(*point, origin)
    np.testing.assert_allclose(geodetic_to_enu(*geo, origin), point, atol=1e-7)


def test_px4_body_and_world_axes():
    p, q, c = px4_pose([1., 2., -3.], [1., 0., 0., 0.], [1., 2., 3.],
                        [.1, .2, .3], [4., 5., 6.])
    np.testing.assert_allclose(p, [6., 6., 9.])
    np.testing.assert_allclose(rotation(q) @ [1., 0., 0.], [0., 1., 0.], atol=1e-10)
    np.testing.assert_allclose(np.diag(c)[:3], [2., 1., 3.])


@pytest.mark.parametrize("latitude,longitude", [(91., 0.), (0., 181.), (float("nan"), 0.)])
def test_invalid_geodesy_rejected(latitude, longitude):
    with pytest.raises(ValueError):
        make_enu_origin(latitude, longitude, 0.)


def perturbation(vector):
    angle = np.linalg.norm(vector)
    return rotation([np.cos(angle/2), *(vector*np.sinc(angle/(2*np.pi))/2)])


def test_joint_pose_covariance_matches_correlated_monte_carlo():
    random = np.random.default_rng(513)
    anchor_rotation = perturbation(np.array([.05, -.1, .6]))
    relative_rotation = perturbation(np.array([-.04, .1, -.2]))
    anchor_position = np.array([3., -2., 4.])
    relative_position = np.array([2., 1., -4.])
    a = random.normal(size=(6, 6)) * np.array([.015]*3 + [.002]*3)[:, None]
    b = random.normal(size=(6, 6)) * np.array([.01]*3 + [.003]*3)[:, None]
    ca, cr = a @ a.T, b @ b.T
    point, attitude, predicted = ugv_pose(
        (anchor_position, quaternion(anchor_rotation), ca), relative_position,
        quaternion(relative_rotation), cr)
    errors = []
    nominal_rotation = rotation(attitude)
    for da, dr in zip(random.multivariate_normal(np.zeros(6), ca, 6000),
                      random.multivariate_normal(np.zeros(6), cr, 6000)):
        ra = perturbation(da[3:]) @ anchor_rotation
        rr = perturbation(dr[3:]) @ relative_rotation
        p = anchor_position + da[:3] + ra @ (relative_position + dr[:3])
        difference = (ra @ rr) @ nominal_rotation.T
        angle_error = np.array([difference[2, 1]-difference[1, 2],
                                difference[0, 2]-difference[2, 0],
                                difference[1, 0]-difference[0, 1]])/2
        errors.append(np.r_[p-point, angle_error])
    empirical = np.cov(np.array(errors).T)
    np.testing.assert_allclose(np.diag(empirical), np.diag(predicted), rtol=.08)
    # The off-diagonal blocks matter when x/y/yaw are fused together.
    np.testing.assert_allclose(empirical[:3, 3:], predicted[:3, 3:], atol=2e-5)


def test_planar_heading_covariance_includes_roll_pitch_and_cross_terms():
    r = perturbation(np.array([.2, .4, 2.9]))
    c = np.diag([.1]*3 + [.01]*3)
    c[0, 5] = c[5, 0] = .005
    q, result = planar_heading_pose(quaternion(r), c)
    assert result[5, 5] > c[5, 5]
    assert result[0, 5] == pytest.approx(.005)
    np.testing.assert_allclose(rotation(q)[:, 0][:2], r[:2, 0]/np.linalg.norm(r[:2, 0]))
    assert np.linalg.eigvalsh(result).min() > 0


def test_planar_heading_rejects_vertical_chassis():
    with pytest.raises(ValueError, match='heading'):
        planar_heading_pose([np.sqrt(.5), 0., np.sqrt(.5), 0.], np.eye(6))
