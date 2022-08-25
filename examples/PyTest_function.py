
# # content of test_sample.py
# def func(x):
#     return x + 1
# def test_answer(my_integer):
#     # assert func(4) == 5
#     assertTrue(1 <= my_integer <= 20)

minimum_number = 0
maximum_number = 23

def get_velocity_x():
    return 0

def get_velocity_y():
    return 10

def get_torque_x():
    return 15

def get_torque_y():
    return 30

def test_velocity_x_equal():
    assert  minimum_number< get_velocity_x()<= maximum_number

def test_velocity_y_equal():
    assert  minimum_number< get_velocity_y()<= maximum_number

def test_torque_x_equal():
    assert  minimum_number< get_torque_x()<= maximum_number

def test_torque_y_equal():
    assert  minimum_number< get_torque_y()<= maximum_number

if __name__ == "__main__":
    test_velocity_x_equal
    test_velocity_y_equal
    test_torque_x_equal
    test_torque_y_equal