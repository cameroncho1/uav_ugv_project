"""Pure numerical geodesy and axis tests do not import ROS."""

import numpy as np
import pytest
from localization_common.geodesy import make_enu_origin, geodetic_to_enu, enu_to_geodetic
from localization_common.frames import px4_pose, rotation


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
