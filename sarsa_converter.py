import csv

csv_file = 'q_table.csv'
output_file = 'q_table_progmem.h'

# Read the Q-table from the CSV file
q_table = []
with open(csv_file, 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip the header
    for row in reader:
        q_table.append([float(value) for value in row[1:]])

# Write the PROGMEM array to a header file
with open(output_file, 'w') as file:
    file.write('#include <Arduino.h>\n')
    file.write('#include <avr/pgmspace.h>\n\n')
    file.write('const uint16_t num_states = {};\n'.format(len(q_table)))
    file.write('const uint8_t num_actions = {};\n\n'.format(len(q_table[0])))
    file.write('const float q_table[num_states][num_actions] PROGMEM = {\n')
    for row in q_table:
        formatted_values = ', '.join(['{: .4f}'.format(val) for val in row])
        file.write('  {{ {} }},\n'.format(formatted_values))
    file.write('};\n')
