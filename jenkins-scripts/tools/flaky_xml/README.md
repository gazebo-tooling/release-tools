These files were generated with the flaky_junit_merge.py script:

~~~
for i in `seq 3`; do
  for j in `seq 3`; do
    python ../flaky_junit_merge.py \
      input$i.xml input$j.xml \
      > output$i$j.xml
  done
done
~~~
