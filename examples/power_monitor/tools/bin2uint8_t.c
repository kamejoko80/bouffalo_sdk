#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

void convert_to_header(const char *input_filename, const char *output_filename) {
    FILE *input_file = fopen(input_filename, "rb");
    if (input_file == NULL) {
        fprintf(stderr, "Error: Cannot open input file %s\n", input_filename);
        exit(1);
    }

    FILE *output_file = fopen(output_filename, "w");
    if (output_file == NULL) {
        fprintf(stderr, "Error: Cannot open output file %s\n", output_filename);
        fclose(input_file);
        exit(1);
    }

    // Get the size of the input file
    fseek(input_file, 0, SEEK_END);
    long file_size = ftell(input_file);
    fseek(input_file, 0, SEEK_SET);

    // Allocate memory to read the file
    uint8_t *buffer = (uint8_t *)malloc(file_size);
    if (buffer == NULL) {
        fprintf(stderr, "Error: Memory allocation failed\n");
        fclose(input_file);
        fclose(output_file);
        exit(1);
    }

    // Read the file into the buffer
    fread(buffer, 1, file_size, input_file);

    // Write the array to the output file
    fprintf(output_file, "#ifndef __GW1N_IMAGE_H__\n");
    fprintf(output_file, "#define __GW1N_IMAGE_H__\n\n");
    fprintf(output_file, "#include <stdint.h>\n\n");
    fprintf(output_file, "const uint8_t gw1n_image[] = {");

    for (long i = 0; i < file_size; ++i) {
        if (i % 12 == 0) {
            fprintf(output_file, "\n    ");
        }
        fprintf(output_file, "0x%02X", buffer[i]);
        if (i < file_size - 1) {
            fprintf(output_file, ", ");
        }
    }

    fprintf(output_file, "\n};\n\n");
    fprintf(output_file, "#endif // __GW1N_IMAGE_H__\n");

    // Clean up
    free(buffer);
    fclose(input_file);
    fclose(output_file);
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <input_binary_file> <output_header_file>\n", argv[0]);
        return 1;
    }

    convert_to_header(argv[1], argv[2]);
    return 0;
}
