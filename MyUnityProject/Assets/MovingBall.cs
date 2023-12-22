using OctopusController;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEditor.SceneManagement;
using UnityEditorInternal;
using UnityEngine;
using UnityEngine.UIElements;

public class MovingBall : MonoBehaviour
{
    [SerializeField]
    IK_tentacles _myOctopus;

    //movement speed in units per second
    [Range(-1.0f, 1.0f)]
    [SerializeField]
    private float _movementSpeed = 5f;

    [SerializeField] private ForceCanvas _forceCanvas;

    [SerializeField] private Transform _onPrefab;
    [SerializeField] private Transform _offPrefab;
    [SerializeField] private Transform _greenArrowPrefab;
    [SerializeField] private Transform _redArrowPrefab;
    [SerializeField] private Transform _whiteArrowPrefab;

    [SerializeField] private Transform _forceArrow;
    [SerializeField] private Transform _velocityArrow;

    public RigidBodyState _state = new RigidBodyState();

    Transform preview;

    List<Vector3> previewPointsMagnus = new List<Vector3>();
    List<Vector3> previewPointsNormal = new List<Vector3>();
    Vector3 goalPoint;

    void Start()
    {
        preview = transform.parent.Find("Preview");
        _state.radius = 0.1f;
        //ApplyForce(new Vector3(0f, 0f, -10000f), new Vector3(-126.775002f, 21.5470009f, -39.1300011f));
    }

    private void OnGUI()
    {
        Event e = Event.current;
        if (!e.isKey || !(e.type == EventType.KeyDown)) return;

        if (e.keyCode == KeyCode.Z)
        {
            _forceCanvas.SetEffectSliderValue(_forceCanvas.GetEffectSliderValue() - 15f);

            if (!preview.gameObject.activeSelf) return;
        }
        else if (e.keyCode == KeyCode.X)
        {
            _forceCanvas.SetEffectSliderValue(_forceCanvas.GetEffectSliderValue() + 15f);

            if (!preview.gameObject.activeSelf) return;
        }
        else if (e.keyCode == KeyCode.I)
        {
            preview.gameObject.SetActive(!preview.gameObject.activeSelf);
        }
    }

    void FixedUpdate()
    {
        if (!_state.update) return;

        _state.UpdateState(transform, true, Time.fixedDeltaTime);

        if (_forceArrow.gameObject.activeSelf && _state.magnusForce != Vector3.zero) 
            _forceArrow.rotation = Quaternion.LookRotation(_state.magnusForce);
        if (_velocityArrow.gameObject.activeSelf && _state.linearMomentum != Vector3.zero)
            _velocityArrow.rotation = Quaternion.LookRotation(_state.linearMomentum);
    }

    public void Shoot(Force force)
    {
        CalculatePreview(force, 100, Time.fixedDeltaTime);
        GeneratePreviewPoints();

        _state.ApplyForce(force);
        _state.update = true;
        _forceArrow.gameObject.SetActive(true);
        _velocityArrow.gameObject.SetActive(true);

        _myOctopus.NotifyShoot(transform, goalPoint);
    }

    private void CalculatePreview(Force force, int pointAmount, float dt)
    {
        CalculatePreviewPoints(previewPointsNormal, _offPrefab, force, false, pointAmount, dt);
        CalculatePreviewPoints(previewPointsMagnus, _onPrefab, force, true, pointAmount, dt);
    }

    private void GeneratePreviewPoints()
    {
        ClearPreview();

        Transform startArrow = Instantiate(_whiteArrowPrefab, preview.transform.position, Quaternion.identity, preview);
        Quaternion r = Quaternion.LookRotation(previewPointsMagnus[0] - startArrow.position);
        startArrow.rotation = r;

        for (int i = 0; i < previewPointsMagnus.Count; i++)
        {
            Instantiate(_onPrefab, previewPointsMagnus[i], Quaternion.identity, preview);
        }
        for (int i = 0; i < previewPointsNormal.Count; i++)
        {
            Instantiate(_offPrefab, previewPointsNormal[i], Quaternion.identity, preview);
        }
    }

    private void CalculatePreviewPoints(List<Vector3> points, Transform arrowPrefab, Force force, bool applyMagnus, int pointAmount, float dt)
    {
        RigidBodyState state = new RigidBodyState();
        state.radius = _state.radius;

        state.ApplyForce(force);

        Transform path = new GameObject().transform;
        path.position = preview.position;
        path.rotation = preview.rotation;

        goalPoint = Vector3.zero;
        for (int i = 0; i < pointAmount; i++)
        {
            state.UpdateState(path, applyMagnus, dt);
            points.Add(path.transform.position);

            if (path.transform.position.z < _myOctopus.transform.position.z && goalPoint == Vector3.zero)
            {
                goalPoint = path.transform.position;
            }
        }

        Destroy(path.gameObject);
    }

    private void ClearPreview()
    {
        for (int i = 0; i < preview.childCount; i++)
        {
            Destroy(preview.GetChild(i).gameObject);
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.GetComponent<NotifyRegion>() == null) return;
        _myOctopus.NotifyStop();
        _state.ApplyForce(new Force(collision.collider.transform.position, (transform.position - collision.collider.transform.position).normalized * 7000f));
    }
}
