using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;


using UI = UnityEngine.UI;

public class april_detector : MonoBehaviour
{
    [SerializeField] Vector2Int _resolution = new Vector2Int(1920, 1080);
    [SerializeField] int _decimation = 4;
    [SerializeField] float _tagSize = 0.05f;
    [SerializeField] Material _tagMaterial = null;
    [SerializeField] UI.RawImage _webcamPreview = null;
    [SerializeField] UI.Text _debugTatget = null;
    public UI.Text _debugBoE = null;
    public UI.Text _debugError = null;

    public GameObject _targetGO;
    public GameObject _BoEGO;
    public GameObject _phoneGO;

    public bool updated;
    public int lin_vel;
    public int ang_vel;

    public float fov_deg = 88.0f;

    // Webcam input and buffer
    WebCamTexture _webcamRaw;
    RenderTexture _webcamBuffer;
    Color32[] _readBuffer;

    // AprilTag detector and drawer
    AprilTag.TagDetector _detector;
    TagDrawer _drawer;

    void Start()
    {
        // Webcam initialization
        _webcamRaw = new WebCamTexture(1920, 1080, 60);
        _webcamRaw.Play();

        _resolution.x = _webcamRaw.width;
        _resolution.y = _webcamRaw.height;

        _webcamBuffer = new RenderTexture(_resolution.x, _resolution.y, 0);
        _readBuffer = new Color32[_resolution.x * _resolution.y];
        _webcamPreview.texture = _webcamBuffer;

        // Detector and drawer
        _detector = new AprilTag.TagDetector(_resolution.x, _resolution.y, _decimation);
        _drawer = new TagDrawer(_tagMaterial);
    }

    void OnDestroy()
    {
        Destroy(_webcamRaw);
        Destroy(_webcamBuffer);

        _detector.Dispose();
        _drawer.Dispose();
    }

    void Update()
    {
        // Check if the webcam is ready (needed for macOS support)
        if (_webcamRaw.width <= 16)
        {
            return;
        }
        // Check if the webcam is flipped (needed for iOS support)
        if (_webcamRaw.videoVerticallyMirrored)
            _webcamPreview.transform.localScale = new Vector3(1, -1, 1);
        // Webcam image buffering
        _webcamRaw.GetPixels32(_readBuffer);
        Graphics.Blit(_webcamRaw, _webcamBuffer);
        // AprilTag detection
        var fov = fov_deg * Mathf.Deg2Rad;
        try
        {
            _detector.ProcessImage(_readBuffer, fov, _tagSize);
        }
        catch
        {
            _debugBoE.text += "EXCEPTION HERE !!!!";
        }
        // Detected tag visualization
        _debugTatget.color = Color.red;
        _debugBoE.color = Color.red;
        int num_detections = _detector.DetectedTags.Count();
        if(num_detections == 2)
        {
            do_control();
        }
        foreach (var tag in _detector.DetectedTags)
        {    
            _drawer.Draw(tag.ID, tag.Position, tag.Rotation, _tagSize);
            if(tag.ID == 3)
            {
                _debugTatget.text = "Got Target in Pos X: " + tag.Position*-1 +  " Rotation: " + tag.Rotation.eulerAngles*-1;
                _debugTatget.color = Color.green;
            }
            if (tag.ID == 7)
            {
                _debugBoE.text = "Got BoE in Pos X: " + tag.Position*-1 + " Rotation: " + tag.Rotation.eulerAngles*-1;
                _debugBoE.color = Color.green;
            }
        }
    }

    void do_control()
    {
        Vector3 Pcr = new Vector3();
        Vector3 Pct = new Vector3();

        Quaternion Rcr = new Quaternion();
        Quaternion Rct = new Quaternion();

        foreach (var tag in _detector.DetectedTags)
        {
            _drawer.Draw(tag.ID, tag.Position, tag.Rotation, _tagSize);
            if (tag.ID == 3)
            {
                Pct = tag.Position;
                Rct = tag.Rotation;
                _targetGO.transform.localPosition = Pct;
                _targetGO.transform.localRotation = Rct;
            }
            if (tag.ID == 7)
            {
                Pcr = tag.Position;
                Rcr = tag.Rotation;
                _BoEGO.transform.localPosition = Pcr;
                _BoEGO.transform.localRotation = Rcr;
            }
        }

        var Tcr = Matrix4x4.TRS(Pcr, Rcr, Vector3.one);
        var Tct = Matrix4x4.TRS(Pct, Rct, Vector3.one);
        var Trc = Tcr.inverse;

        var Trt = Trc * Tct;

        Vector3 Prt = new Vector3(Trt.m03, Trt.m13, Trt.m23);

        float dist = (Prt.x * Prt.x) + (Prt.y + Prt.y);
        dist = Mathf.Sqrt(dist);

        float ang = Mathf.Atan2(Prt.x, Prt.y);

        _debugError.text = "Error X: " + Prt.x.ToString() + "\nY: " + Prt.y.ToString() + "\nZ: " + (ang * 180 / 3.14).ToString() +
            "\n Dist: " + dist.ToString();

        updated = true;
        lin_vel = (int)(dist*100);
        ang_vel = (int)(ang * 180 / 3.14);

        _targetGO.transform.SetParent(_phoneGO.transform);

    }
}
