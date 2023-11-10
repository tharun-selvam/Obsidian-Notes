
# Lecture 2
- We do not produce discrete values as output because computers don't handle them well and we do not know if given model is going in the right direction (after some tuning) because the output is all or nothing (take the example of dog or cat classifier)
- So what we do is find a prob. distribution of the outputs.
- We take all the outputs received and apply softmax func. to convert the numbers to a positive number and then make the 2 sum to 1.
