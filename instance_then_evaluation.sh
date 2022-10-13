rm -r /workspace/method_predictions/sequences/08
cp -avr /workspace/method_predictions/sequences/08_copy   /workspace/method_predictions/sequences/08
python /workspace/semantic_then_instance_post_inferece.py 
bash evaluation_panoptic.sh