#!/usr/bin/env python3
from argparse import ArgumentParser
import concurrent.futures
from copy import deepcopy
import itertools
import concurrent

from racer.constants import framerate
from racer.game_state import GameState
from racer.track import Track
from racer.tracks import track1, track2

from racer.bots.lewie import Gonzales
from racer.car_info import CarInfo, Car
from racer.cars import car1

MAX_ROUNDS = 3
TRACKS = [track1, track2]


def is_done(car_info):
    return car_info.round == MAX_ROUNDS


# CURRENT BEST
# EFFECTIVE_DECELERATION = -108  # pixels/sec2
# STEER_DISTANCE_LIMIT = 49  # pixels
# STEER_ANGLE_LIMIT = 36  # degrees
# SPEED_LIMIT_FACTOR = 67  # pixels/sec ratio
# SPEED_LIMIT_OFFSET = 90  # pixels/sec
# CORNER_CUT_FACTOR = 53  # percent
# STEERING_FACTOR = 55  # percent

BENCHMARK_TIMES = [84.517, 75.700]


def main():
    r_ed = range(-108, -111, -1)
    r_sdl = range(47, 50, 1)
    r_sal = range(34, 39, 1)
    r_slf = range(66, 69, 1)
    r_slo = range(88, 91, 1)
    r_ccf = range(51, 54, 1)
    r_sf = range(55, 58, 1)

    arg_product = [{
        "effective_deceleration": ed,
        "steer_distance_limit": sdl,
        "steer_angle_limit": sal,
        "speed_limit_factor": slf,
        "speed_limit_offset": slo,
        "corner_cut_factor": ccf,
        "steering_factor": sf
    } for ed, sdl, sal, slf, slo, ccf, sf in itertools.product(
        r_ed, r_sdl, r_sal, r_slf, r_slo, r_ccf, r_sf)]

    print(f"RUNNING {len(arg_product)} games")

    results = []

    count = 0
    total = len(arg_product)

    # sequential
    # for a in arg_product:
    #     args, time = single_game(args=a)
    #     count += 1
    #     if time != 0: results.append((args, time))
    #     print(f"{count}/{total} ({100*count/total:.1f}%): {args} ===> {time}")

    # parallel
    with concurrent.futures.ProcessPoolExecutor(max_workers=16) as executor:
        games = {executor.submit(single_game, args=a): a for a in arg_product}

        for f in concurrent.futures.as_completed(games):
            count += 1
            args, times = f.result()

            if all(t != 0 for t in times): results.append((args, times))
            times_str = " - ".join([f"{float(t):.3f}" for t in times])
            print(
                f"{count}/{total} ({100*count/total:.1f}%): {args} ===> {times_str}"
            )

    print("\n\nALL DONE\n\n")

    for i, t in enumerate(TRACKS):
        print(f"\nTRACK {i}: {t.name}")
        top_results = [r for r in sorted(results, key=lambda x: x[1][i])][0:10]

        for args, times in top_results:
            times_str = " - ".join([f"{float(t):.3f}" for t in times])
            print(f"{args} ===> {times_str}")

    relative_results = [
        (args, [times[0] - BENCHMARK_TIMES[0], times[1] - BENCHMARK_TIMES[1]])
        for args, times in results
    ]

    print("\nRELATIVE RESULTS")

    for args, times in sorted(relative_results,
                              key=lambda x: x[1][0] + x[1][1])[0:20]:
        times_str = " - ".join([f"{float(t):.3f}" for t in times])
        print(f"{args} ===> {times_str}")


def single_game(*, args={}):
    frames_after_finish = 25_000

    times = []
    for t in TRACKS:
        game_state = GameState(Track(t))

        bot = Gonzales(deepcopy(game_state.track), **args)
        game_state.bots.clear()
        game_state.bots[bot] = CarInfo(Car.from_module(car1, bot.color),
                                       game_state.track)

        time = 0
        for f in range(0, frames_after_finish):
            game_state.update(1 / framerate)
            car_info = next(iter(game_state.bots.values()))
            if is_done(car_info):
                time = (f + 1) / framerate
                break
        times.append(time)

    return args, times


if __name__ == '__main__':
    parser = ArgumentParser(
        description='Run a tournament of the coding challenge racer')
    args = parser.parse_args()

    try:
        main(**vars(args))
    except KeyboardInterrupt:
        pass
