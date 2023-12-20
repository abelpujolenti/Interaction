using UnityEngine;

public class Route : MonoBehaviour
{
    [SerializeField] private Transform[] _points;
    [SerializeField] private Transform[] _firstTangents;
    [SerializeField] private Transform[] _secondTangents;
    [SerializeField] private Transform _scorpionBodyPosition;

    [SerializeField] private float maxTimeAnimation;

    private Vector3[][] _exteriorSegmentPoints;
    private Vector3[][] _midSegmentPoints;
    private Vector3[] _interiorSegmentPoints;
    
    private float _timeLerp;

    private bool _play;

    private int _currentCurve;
    private int _maxCurves;
    
    // Start is called before the first frame update
    void Start()
    {
        _scorpionBodyPosition.position = _points[0].position;
        _exteriorSegmentPoints = new Vector3[_points.Length - 1][];
        _midSegmentPoints = new Vector3[_points.Length - 1][];
        _interiorSegmentPoints = new Vector3[_points.Length - 1];

        for (int i = 0; i < _exteriorSegmentPoints.Length; i++)
        {
            _exteriorSegmentPoints[i] = new Vector3[3];
            _midSegmentPoints[i] = new Vector3[2];
        }

        _maxCurves = _points.Length - 1;
    }

    private void Update()
    {
        if (!_play)
        {
            if (!Input.GetKeyDown(KeyCode.Space))
            {
                return;
            }
            _currentCurve = 0;
            _play = true;
            return;
        }

        _timeLerp += (Time.deltaTime / maxTimeAnimation) * _maxCurves;
        
        BezierCurve();

        if (_timeLerp < 1)
        {
            return;
        }

        _timeLerp = 0;
        _currentCurve++;

        if (_currentCurve == _maxCurves)
        {
            _play = false;
        }
    }

    private void BezierCurve()
    {
        Debug.DrawLine(_points[_currentCurve].position, _firstTangents[_currentCurve].position, Color.white);
        Debug.DrawLine(_firstTangents[_currentCurve].position, _secondTangents[_currentCurve].position, Color.white);
        Debug.DrawLine(_secondTangents[_currentCurve].position, _points[_currentCurve + 1].position, Color.white);
        
        
        ExteriorSegments();
        
        MidSegments();
        
        InteriorSegments();

        _scorpionBodyPosition.position = _interiorSegmentPoints[_currentCurve];
    }

    private void ExteriorSegments()
    {
        _exteriorSegmentPoints[_currentCurve][0] = Vector3.Lerp(_points[_currentCurve].position,
            _firstTangents[_currentCurve].position, _timeLerp);
        
        _exteriorSegmentPoints[_currentCurve][1] = Vector3.Lerp(_firstTangents[_currentCurve].position,
            _secondTangents[_currentCurve].position, _timeLerp);
        
        _exteriorSegmentPoints[_currentCurve][2] = Vector3.Lerp(_secondTangents[_currentCurve].position,
            _points[_currentCurve + 1].position, _timeLerp);
        
        Debug.DrawLine(_exteriorSegmentPoints[_currentCurve][0], _exteriorSegmentPoints[_currentCurve][1], Color.green);
        Debug.DrawLine(_exteriorSegmentPoints[_currentCurve][1], _exteriorSegmentPoints[_currentCurve][2], Color.green);
    }

    private void MidSegments()
    {
        _midSegmentPoints[_currentCurve][0] = Vector3.Lerp(_exteriorSegmentPoints[_currentCurve][0],
            _exteriorSegmentPoints[_currentCurve][1], _timeLerp);
        
        _midSegmentPoints[_currentCurve][1] = Vector3.Lerp(_exteriorSegmentPoints[_currentCurve][1],
            _exteriorSegmentPoints[_currentCurve][2], _timeLerp);
        
        Debug.DrawLine(_midSegmentPoints[_currentCurve][0], _midSegmentPoints[_currentCurve][1], Color.red);
    }

    private void InteriorSegments()
    {
        _interiorSegmentPoints[_currentCurve] = Vector3.Lerp(_midSegmentPoints[_currentCurve][0],
            _midSegmentPoints[_currentCurve][1], _timeLerp);
    }
}