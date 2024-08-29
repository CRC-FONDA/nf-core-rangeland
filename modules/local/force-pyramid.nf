process FORCE_PYRAMID {
    tag { tile }
    label 'process_low'

    // outLabel { tile }

    container "docker.io/davidfrantz/force:3.7.10"

    input:
    tuple val(tile), path(image)

    output:
    path('**')         , emit: trends
    path "versions.yml", emit: versions

    when:
    task.ext.when == null || task.ext.when

    script:
    """
    files="*.tif"
    for file in \$files; do
        force-pyramid \$file
    done;

    cat <<-END_VERSIONS > versions.yml
    "${task.process}":
        force: \$(force -v | sed 's/.*: //')
    END_VERSIONS
    """

}
