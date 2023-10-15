- [Resource](https://realpython.com/python-classes/#getting-started-with-python-classes)

## Basic Defintions
1. Attributes: refer to the properties or data associated with a specific object of a given class.
2. Methods: refer to the different behaviors that objects will show. Methods are functions that you define within a class. These functions typically operate on or with the attributes of the underlying instance or class.
3. Members: Attributes + Methods


## Defining a Class in Python
Example:

```python
# circle.py

import math

class Circle:
    def __init__(self, radius):
        self.radius = radius

    def calculate_area(self):
        return round(math.pi * self.radius ** 2, 2)
```

- `.__init__()` method is known as object initializer because ot sets and defines the initial values for your attributes.

## Creating objects 

```python
from circle import Circle

circle_1 = Circle(42)
circle_2 = Circle(7)
```

- The class constructor accepts the same arguments as the `.__init__()` method.

##  Accessing Attributes and Methods
```python
obj.attribute_name

obj.method_name()
```

## Naming Conventions

### Public vs Non-Public Members

|Member|Naming|Examples|
|---|---|---|
|Public|Use the normal naming pattern.|`radius`, `calculate_area()`|
|Non-public|Include a leading underscore in names.|`_radius`, `_calculate_area()`|

- Though we can access non-public members and change theire values, we shouldn't.

### Name Mangling
- Adding two leading underscores creates an automatic name transformation that prepends the class’s name to the member’s name, like in `_ClassName__attribute` or `_ClassName__method`.
- If method name was `__method()` then `class_instance.__method()` gives an error because the `__method()` function got it's name mangled to `class_instance._ClassName__method()`

## Attaching Data to Classes and Instances
1. **Class attributes:** A class attribute is a variable that you define in the class body directly. Class attributes belong to their containing class. Their data is common to the class and all its instances.
2. **Instance attributes:** An instance is a variable that you define inside a method. Instance attributes belong to a concrete instance of a given class. Their data is only available to that instance and defines its state.
### Class Attributes
- You can _access_ class attributes using either the class or one of its instances.
- To _modify_ a class attribute, you must use the class itself rather than one of its instances.
- You can’t modify class attributes through instances of the containing class. Doing that will create new instance attributes with the same name as the original class attributes.
```python
class ObjectCounter:
    num_instances = 0
    def __init__(self):
        ObjectCounter.num_instances += 1
```
### Instance Attributes
- We define instance attributes inside **instance methods**, which are those methods having `self` as their first argument.

>Even though you can define instance attributes inside any instance method, it’s best to define all of them in the `.__init__()` method, which is the instance initializer. This ensures that all of the attributes have the correct values when you create a new instance. Additionally, it makes the code more organized and easier to debug.

```python
# car.py

class Car:
    def __init__(self, make, model, year, color):
        self.make = make
        self.model = model
        self.year = year
        self.color = color
        self.started = False
        self.speed = 0
        self.max_speed = 200
```

- The instance attributes must be defined with the `self` argument.
## Miscellaneous
1. **vars():** In the above example, you use the built-in [`vars()`](https://docs.python.org/3/library/functions.html#vars) function, which returns a dictionary of all the members associated with the given object. This dictionary plays an important role in Python classes.
2. **Note:** Even though you can define instance attributes inside any instance method, it’s best to define all of them in the `.__init__()` method, which is the instance initializer. This ensures that all of the attributes have the correct values when you create a new instance. Additionally, it makes the code more organized and easier to debug.
