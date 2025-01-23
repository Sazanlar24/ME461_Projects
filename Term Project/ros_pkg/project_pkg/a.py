import random

def create_cell_values(cell_number):
    cell_value_string = ""
    list_numbers = [i for i in range(1, cell_number+1)]
    a = str(list_numbers)
    print(a)

    while len(list_numbers) > 1:
        number = random.choice(list_numbers)
        cell_value_string += str(number)
        cell_value_string += ","
        list_numbers.remove(number)
    cell_value_string += str(list_numbers[0])
    print(cell_value_string)

    

create_cell_values(3)