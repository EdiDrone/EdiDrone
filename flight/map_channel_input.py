'''
Map input on [-1,1] to piecewise-linear range, which linearly maps [-1,0) to [min, neutral)
  and [0,1] to [neutral, max]
'''
def map_channel_to_output(input_value, min, neutral, max):
    # Scale input to bi-linear range
    if input_value >= 0.0:
        value_mapped = input_value * (max-neutral) + neutral
    else:
        value_mapped = input_value * (neutral-min) + neutral

    # Saturate result
    if max > min:
        if value_mapped > max:
            value_mapped = max
        if value_mapped < min:
            value_mapped = min
    else:
        if value_mapped < max:
            value_mapped = max
        if value_mapped > min:
            value_mapped = min

    return value_mapped

'''
Map piecewise-linear input to output in [-1,1], which linearly maps [-1,0) to [min, neutral)
  and [0,1] to [neutral, max]
'''
def map_input_to_channel(input_value, min, neutral, max):

    # Scale input to bi-linear range
    if (max > min and input_value >= neutral) or (min > max and input_value <= neutral):
        if max != neutral:
            value_mapped = (input_value - neutral) / float(max - neutral)
        else:
            value_mapped = 0.0
    else:
        if min != neutral:
            value_mapped = (input_value - neutral) / float(neutral - min)
        else:
            value_mapped = 0.0

    # Bound
    if value_mapped >  1.0:
        value_mapped =  1.0
    elif value_mapped < -1.0:
        value_mapped = -1.0

    return value_mapped
