using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class falllDetector : MonoBehaviour
{
    // Start is called before the first frame update
    Vector3 accelVector;
    public Text accelText;

    public int fallFlag = 0;
    int freefall_counter = 0;
    int freefall_threshold = 5;
    float accel_norm_threshold = 0.05f;

    float lowAccel = 1000.0f;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        accelVector = Input.acceleration; // Get the acceleration vector from the IMU
        float magAccel = accelVector.magnitude; // Computing the norm of the vector
        if (magAccel < lowAccel)
            lowAccel = magAccel; // Debugging to see the lowest norms
        if (fallFlag == 0)
            accelText.text = lowAccel.ToString();
        
        if (magAccel < accel_norm_threshold) // Check if the magnitude is lower than the threshold
        {
            freefall_counter += 1; // Update a variable if lower
            if (freefall_counter > freefall_threshold) // Only sets the fall flag if the norm is low for n number of frames
            {
                fallFlag = 1; // Sets the fall flag
                accelText.text = "FALL DETECTED";
            }

        }
        else
        {
            freefall_counter = 0; // resets the counter if norm is high, used for filtering
        }

        
    }
    // Resets the fall flag
    public void resetFallFlag()
    {
        fallFlag = 0;
        accelText.text = "";
    }
}
