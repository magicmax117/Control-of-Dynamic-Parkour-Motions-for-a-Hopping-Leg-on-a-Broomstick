import numpy as np


def polynom_generator(PARKOUR):
    obstacle_polys = [None] * len(PARKOUR["position"])
    hurdle_polys = [None] * len(PARKOUR["position"])
    platform_polys = [None] * len(PARKOUR["position"])

    for i in range(len(PARKOUR["position"])):
        # create obstacle
        points_x = [PARKOUR["position"][i] - PARKOUR["width"][i] / 2,
                    PARKOUR["position"][i],
                    PARKOUR["position"][i] + PARKOUR["width"][i] / 2]
        points_y = [0, PARKOUR["height"][i], 0]
        coefficients = np.polyfit(points_x, points_y, 2)
        obstacle_polys[i] = np.poly1d(coefficients)

        # create hurdle
        points_x = [PARKOUR["position"][i] - PARKOUR["width"][i] / 2 - PARKOUR["hurdle_margin_front"],
                    PARKOUR["position"][i] + (PARKOUR["hurdle_margin_back"] - PARKOUR["hurdle_margin_front"]) / 2,
                    PARKOUR["position"][i] + PARKOUR["width"][i] / 2 + PARKOUR["hurdle_margin_back"]]
        points_y = [0, PARKOUR["height"][i], 0]
        coefficients = np.polyfit(points_x, points_y, 2)
        hurdle_polys[i] = np.poly1d(coefficients)

        # create platform
        points_x = [PARKOUR["position"][i] - PARKOUR["width"][i] / 2 + PARKOUR["platform_margin"],
                    PARKOUR["position"][i],
                    PARKOUR["position"][i] + PARKOUR["width"][i] / 2 - PARKOUR["platform_margin"]]
        points_y = [0, PARKOUR["height"][i], 0]
        coefficients = np.polyfit(points_x, points_y, 2)
        platform_polys[i] = np.poly1d(coefficients)

    return obstacle_polys, hurdle_polys, platform_polys
