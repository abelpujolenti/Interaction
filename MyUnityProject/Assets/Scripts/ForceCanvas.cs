using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ForceCanvas : MonoBehaviour
{
    [SerializeField] Slider _forceSlider;
    [SerializeField] Slider _effectSlider;

    public void SetForceSliderValue(float value) { 
        _forceSlider.value = value;
    }

    public float GetForceSliderValue() { 
        return _forceSlider.value;
    }

    public void SetEffectSliderValue(float value) { 
        _effectSlider.value = value;
    }

    public float GetEffectSliderValue()
    {
        return _effectSlider.value;
    }
}
