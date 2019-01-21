using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ReadCSV : MonoBehaviour
{
    private string fieldSeperater = ";";
    private string lineSeperater = "\n";
    private string fileData;
    private string path;
    private string [] lines;
    private string[] lineHeader;
    private float x;

    void Start()
    {
        path = Application.dataPath + "/SavedData.csv";
        fileData = System.IO.File.ReadAllText(path);
        lines = fileData.Split("\n"[0]);
        lineHeader = (lines[0].Trim()).Split(fieldSeperater[0]);
        printLine(0);
        
    }

    private void printLine(int index)
    {
        string output = string.Join(", ",
             new List<string>(getLineValues(index))
             .ConvertAll(i => i.ToString())
             .ToArray());
        Debug.Log(output);
    }

    public string[] getLineValues(int index)
    {
        string[] line = (lines[index].Trim()).Split(fieldSeperater[0]);
        return line;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
