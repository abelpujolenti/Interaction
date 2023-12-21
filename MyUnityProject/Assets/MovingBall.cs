using System.Collections;
using System.Collections.Generic;
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

    RigidBodyState _state = new RigidBodyState();

    Transform preview;

    void Start()
    {
        preview = transform.parent.Find("Preview");
        _state.radius = GetComponent<SphereCollider>().radius * 10f;
        //ApplyForce(new Vector3(0f, 0f, -10000f), new Vector3(-126.775002f, 21.5470009f, -39.1300011f));
    }

    private void OnGUI()
    {
        Force force = new Force(new Vector3(-126.775002f, 21.7f, -39.1300011f), new Vector3(0f, 0f, -10000f));
        Event e = Event.current;
        if (!e.isKey || !(e.type == EventType.KeyDown)) return;

        if (e.keyCode == KeyCode.Z)
        {
            _forceCanvas.SetEffectSliderValue(_forceCanvas.GetEffectSliderValue() - 15f);
        }
        else if (e.keyCode == KeyCode.X)
        {
            _forceCanvas.SetEffectSliderValue(_forceCanvas.GetEffectSliderValue() + 15f);
        }
        else if (e.keyCode == KeyCode.I)
        {
            preview.gameObject.SetActive(!preview.gameObject.activeSelf);
            GeneratePreview(force, 100, Time.fixedDeltaTime);
        }
        else if (e.keyCode == KeyCode.S)
        {
            _state.ApplyForce(force);
            _state.update = true;
        }
    }

    void FixedUpdate()
    {
        if (!_state.update) return;

        _state.UpdateState(transform, true, Time.fixedDeltaTime);
    }

    private void GeneratePreview(Force force, int arrowAmount, float dt)
    {
        ClearPreview();
        GeneratePreviewArrows(_onPrefab, force, true, arrowAmount, dt);
        GeneratePreviewArrows(_offPrefab, force, false, arrowAmount, dt);
    }

    private void GeneratePreviewArrows(Transform arrowPrefab, Force force, bool applyMagnus, int arrowAmount, float dt)
    {
        RigidBodyState state = new RigidBodyState();
        state.radius = _state.radius;
        state.ApplyForce(force);
        Transform currArrow = Instantiate(arrowPrefab, preview.transform.position, Quaternion.identity, preview);
        Transform lmao = new GameObject().transform;
        lmao.position = preview.position;
        lmao.rotation = preview.rotation;
        for (int i = 0; i < arrowAmount; i++)
        {
            state.UpdateState(lmao, applyMagnus, dt);
            Transform prevArrow = currArrow;
            currArrow = Instantiate(arrowPrefab, lmao.transform.position, Quaternion.identity, preview);
            //Quaternion r = Quaternion.LookRotation(currArrow.position - prevArrow.position);
            //currArrow.rotation = r;
        }
        Destroy(lmao.gameObject);
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
        _myOctopus.NotifyShoot();
    }
}
