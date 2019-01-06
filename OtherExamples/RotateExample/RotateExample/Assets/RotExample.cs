using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotExample : MonoBehaviour {

	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
        float deltaOmegay = Time.deltaTime * 360f / 5f;
        Debug.Log(deltaOmegay);
        transform.Rotate(0, deltaOmegay, 0);
    }
}
