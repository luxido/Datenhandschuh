using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text.RegularExpressions;

public class ReadCSV : MonoBehaviour
{
    private char fieldSeperater = ';';  //or ','
    private string lineSeperater = "\r\n";
    private string fileData;
    private string path;
    private string[] lines;
    private string[] lineHeader;
    private float x;

    public void init(string path)
    {
        fileData = System.IO.File.ReadAllText(path);
        //lines = fileData.Split(lineSeperater);
        lines = Regex.Split(fileData, lineSeperater);
        lineHeader = (lines[0].Trim()).Split(fieldSeperater);
        //print header
        PrintLine(0);
    }

    public int CountLines()
    {
        return lines.GetLength(0);
    }

    private void PrintLine(int index)
    {
        string output = string.Join(", ",
             new List<string>(GetLineValues(GetLine(index)))
             .ConvertAll(i => i.ToString())
             .ToArray());
        Debug.Log(output);
    }

    public string[] GetLineValues(string line)
    {
        string[] values = line.Split(fieldSeperater);
        return values;
    }

    public string GetLine(int index)
    {
        return lines[index];
    }

    public int GetIndexOf(string name)
    {
        int index = 0;
        for (int i = 0; i < lineHeader.Length; i++)
        {
            if (lineHeader[i].Equals(name))
            {
                index = i;
                break;
            }
        }
        return index;
    }
}
