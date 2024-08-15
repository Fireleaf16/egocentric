import random

def random_selection():
    numbers = list(range(1, 7))  # Initialize the list with numbers 1 to 6
    
    while numbers:  # Continue until the list is empty
        selected = random.choice(numbers)  # Randomly select a number from the list
        print(f"Selected number: {selected}")
        
        numbers.remove(selected)  # Remove the selected number from the list
        
        if not numbers:  # Check if the list is empty
            print("All numbers have been selected.")


# Run the function
random_selection()
