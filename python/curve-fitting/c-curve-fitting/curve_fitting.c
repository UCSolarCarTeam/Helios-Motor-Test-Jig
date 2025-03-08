#include <stdio.h>
#include <math.h>

// Define the function to calculate the DC current based on Torque (x) and Speed (y)
double predict_dc_current(double x, double y, double x_mean, double x_std, double y_mean, double y_std, double coefficients[]) {
    // Normalize the input values
    double x_norm = (x - x_mean) / x_std;
    double y_norm = (y - y_mean) / y_std;

    // Calculate each term of the polynomial function
    double z = coefficients[0] +
               coefficients[1] * x_norm +
               coefficients[2] * y_norm +
               coefficients[3] * pow(x_norm, 2) +
               coefficients[4] * pow(y_norm, 2) +
               coefficients[5] * x_norm * y_norm +
               coefficients[6] * pow(x_norm, 3) +
               coefficients[7] * pow(y_norm, 3) +
               coefficients[8] * pow(x_norm, 2) * y_norm +
               coefficients[9] * x_norm * pow(y_norm, 2) +
               coefficients[10] * pow(x_norm, 4) +
               coefficients[11] * pow(y_norm, 4) +
               coefficients[12] * pow(x_norm, 3) * y_norm +
               coefficients[13] * pow(x_norm, 2) * pow(y_norm, 2) +
               coefficients[14] * x_norm * pow(y_norm, 3) +
               coefficients[15] * pow(x_norm, 5) +
               coefficients[16] * pow(y_norm, 5) +
               coefficients[17] * pow(x_norm, 4) * y_norm +
               coefficients[18] * pow(x_norm, 3) * pow(y_norm, 2) +
               coefficients[19] * pow(x_norm, 2) * pow(y_norm, 3) +
               coefficients[20] * x_norm * pow(y_norm, 4) +
               coefficients[21] * pow(x_norm, 6) +
               coefficients[22] * pow(y_norm, 6);

    return z;
}

int main() {
    // Define the dataset statistics
    double x_mean = 72.63720930232559;    //update this value from the dataset being stored
    double x_std = 42.52505156747794;    //update this value from the dataset being stored
    double y_mean = 469.7674418604651;  //update this value from the dataset being stored
    double y_std = 300.12391527727823; //update this value from the dataset being stored

    // Coefficients from the model
    double coefficients[] = {
        8.99081610e+01,  5.41331468e+01,  5.42552952e+01,  3.52967650e+00,
        -1.39293173e+00,  3.12836964e+01, -3.00255167e-01,  1.45315850e+00,
        1.91373226e+00,  2.22332187e+00, -1.06146310e-01,  2.61885344e+00,
        1.89991053e-01,  1.15873486e+00,  1.74216668e+00,  2.55780027e-01,
        1.91376837e-01, -1.32832021e-01, -6.02793556e-02,  4.50207064e-01,
        1.49791194e-02, -6.62227062e-02, -5.02206451e-01
    };

    // Example inputs
    double x = 25;   // Torque [Nm]
    double y = 300.0; // Speed [rpm]

    // Get the predicted current
    double predicted_current = predict_dc_current(x, y, x_mean, x_std, y_mean, y_std, coefficients);

    // Print the result
    printf("Predicted DC Current: %f A\n", predicted_current);

    return 0;
}