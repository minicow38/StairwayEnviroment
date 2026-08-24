Shader "Custom/MoyaTrailURP"
{
    Properties
    {
        _BaseMap ("Texture", 2D) = "white" {}
        _BaseColor ("Tint", Color) = (0.65, 0.9, 1.0, 0.28)
        _EmissionStrength ("Glow", Range(0, 4)) = 0.8
        _Alpha ("Opacity", Range(0, 1)) = 0.72
        _SoftPower ("Soft Edge", Range(0.4, 2.5)) = 1.15
    }

    SubShader
    {
        Tags
        {
            "RenderType"="Transparent"
            "Queue"="Transparent"
            "RenderPipeline"="UniversalPipeline"
        }

        Pass
        {
            Name "MoyaTrail"
            Tags { "LightMode"="UniversalForward" }

            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off
            Cull Off

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            struct Attributes
            {
                float4 positionOS : POSITION;
                float2 uv : TEXCOORD0;
                float4 color : COLOR;
            };

            struct Varyings
            {
                float4 positionHCS : SV_POSITION;
                float2 uv : TEXCOORD0;
                float4 color : COLOR;
            };

            TEXTURE2D(_BaseMap);
            SAMPLER(sampler_BaseMap);

            CBUFFER_START(UnityPerMaterial)
                float4 _BaseMap_ST;
                float4 _BaseColor;
                float _EmissionStrength;
                float _Alpha;
                float _SoftPower;
            CBUFFER_END

            Varyings vert(Attributes IN)
            {
                Varyings OUT;
                OUT.positionHCS = TransformObjectToHClip(IN.positionOS.xyz);
                OUT.uv = TRANSFORM_TEX(IN.uv, _BaseMap);
                OUT.color = IN.color;
                return OUT;
            }

            half4 frag(Varyings IN) : SV_Target
            {
                half4 tex = SAMPLE_TEXTURE2D(_BaseMap, sampler_BaseMap, IN.uv);

                // Alpha from the soft texture; curve it slightly for a mistier edge
                half a = pow(saturate(tex.a), _SoftPower);
                a *= _BaseColor.a * IN.color.a * _Alpha;

                // Slightly luminous but still alpha-blended, so it feels like air rather than a laser.
                half3 rgb = tex.rgb * _BaseColor.rgb * IN.color.rgb;
                rgb *= (1.0h + _EmissionStrength);

                return half4(rgb, a);
            }
            ENDHLSL
        }
    }
}
