
force = [-1, -2, -3, -4, -5, 0, 1, 2, 3, 4, 5]
torque = [11, 12, 13, 14, 15, 16]

def test_force_x_range():
    for x in force:
        try:
            assert 0 < x <= 10
        except:
            print(f"force {x} in x not in the range  ")


def test_torque_x_range():
    for x in torque:
        try:
            assert 0 < x <= 10
        except:
            print(f"torque {x} in x not in the range  ")


# if __name__ == "__main__":
#
#     test_force_x_range()
#     test_torque_x_range()


