prompt_text = '''
We are attempting to utilize a reinforcement learning + S-Surface controller (a nonlinear controller functionally analogous to PID) workflow for 6-DOF attitude control of a UUV. The reinforcement learning output will directly replace the error input in the controller, enabling the PID output to better meet control performance requirements. Based on the execution log below, adjust the UUV actuator parameters.

## S-Surface Controller

The S-Surface controller shares identical inputs with a PD controller: the observation error *e* and its derivative. However, the S-Surface controller is nonlinear. Specifically, the relationship between the control signal (denoted as *u*) and the error (denoted as *e*) is given by:

$$
u = \frac{2}{1 + \exp(-\zeta_1 e - \zeta_2 \dot{e})} - 1
$$

where $\zeta_1$ and $\zeta_2$ are controller parameters.

- $\zeta_1$ is the most critical factor determining response.
- $\zeta_2$ serves auxiliary regulation. It is not primarily adjusted.

## Control Log

The controller's response curves from the last three parameter adjustments are displayed in the image, illustrating control performance for roll, pitch, and yaw angles (enabling necessary angle control as applicable). Interpret the log to understand controller effectiveness holistically and in detail. Additionally, some references are provided below.

Historical MSE value (reflecting tracking error, rad^2): <mse_value>

Historical controller parameters are also shown: <final_pid_value>

Controlled angles: roll (<roll_control>) / pitch (<pitch_control>) / yaw (<yaw_control>)

## Output Requirements

Consequently, in your response, consider the following factors:

- **Fundamental verification**: Infer from historical data how many parameter adjustments have occurred.
- Analyze control performance for all enabled degrees of freedom, incorporating reference signal characteristics. Perform both qualitative and quantitative analysis based on the image.
- Explain prior PID parameter adjustments and their effects on controller feedback.
- Specify required PID parameter adjustments to improve control performance.

After this rationale, output **only** a Python dictionary using the format below. For each key, provide the **multiplicative factor** relative to previous parameters (e.g., 1.25 for a 25% increase):

```python
{'roll_zeta1':1.25, 'roll_zeta2':1,'pitch_zeta1':1, 'pitch_zeta2':1, 'yaw_zeta1':1, 'yaw_zeta2':1}
```

**Your response must consist solely of this single Python code block; no other content should be output.**

When determining parameter adjustments, consider:
- $\zeta_1$ is the primary factor for response speed and overshoot. Adjust $\zeta_1$ first during initial tuning.
- $\zeta_2$ is a secondary factor for overshoot; excessively high or low values degrade performance:
  - If significant overshoot persists after adjusting $\zeta_1$, increase $\zeta_2$.
  - If performance remains poor after increasing $\zeta_2$, decrease $\zeta_2$.

Select multiplicative factors from these primary stages (fine adjustments permitted):
- 2: Severe parameter deviation requiring substantial adjustment.
- 1.5: Standard parameter increase.
- 1: Parameter unchanged.
- 0.67: Standard parameter decrease.
- 0.5: Significant parameter reduction.

Critical notes:
- Actuator output saturation imposes upper limits; avoid further increasing $\zeta_1$ if historical adjustments yielded negligible improvement.
- Since reinforcement learning + controller integration is employed, response curves cannot be solely attributed to controller parameters; consider RL output characteristics at low errors (evaluate responses holistically).
- During experiments, certain degrees of freedom may be disabled (e.g., yaw, pitch, or roll control disabled). In such cases, historical PID parameters may omit entries. **Only output parameters for enabled control angles.**
- Focus exclusively on enabled control directions; disabled axes must be ignored.
'''