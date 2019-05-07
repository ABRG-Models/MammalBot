<?xml version="1.0" encoding="ISO-8859-1"?><xsl:stylesheet version="1.0"
xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
xmlns:fn="http://www.w3.org/2005/xpath-functions" exclude-result-prefixes="fn">
<xsl:output method="xml" omit-xml-declaration="no" version="1.0" encoding="UTF-8" indent="yes"/>

<!--
     This is a very brief example of how to link a SpineML model to a Python component and pass data between them
-->

<xsl:param name="spineml_output_dir" select="'./'"/>

<!-- START TEMPLATE -->
<xsl:template name="external">

<xsl:comment>Test comment</xsl:comment>

<!-- GET A LINK TO THE EXPERIMENT FILE FOR LATER USE -->
<xsl:variable name="expt_root" select="/"/>

<!-- GET THE SAMPLE RATE -->
<xsl:variable name="sampleRate" select="(1 div number($expt_root//@dt)) * 1000.0"/>

<!-- This is the destination process -->
<Process>
	<Name>Python_example</Name>
	<Class>dev/python/example</Class>
	<State c="z" a="output_data_path;simtk_integrator;" Format="DataML" Version="5" AuthTool="SystemML Toolbox" AuthToolVersion="0">
		<m><xsl:value-of select="$spineml_output_dir"/></m>
		<m>ExplicitEuler</m>
	</State>
	<Time><SampleRate><xsl:value-of select="$sampleRate"/></SampleRate></Time>
</Process>

<!-- Here, we link each muscle input activity process to the Saccade simulator. -->
<Link>
	<Src>Neuron&gt;neuron_out</Src>
	<Dst>Python_example&lt;&lt;&lt;python_in</Dst>
	<Lag>0</Lag>
</Link>


<Link>
	<Src>Python_example&gt;python_out</Src>
	<Dst>Neuron&lt;&lt;&lt;neuron_in</Dst>
	<Lag>1</Lag>
</Link>

<!--
<Link>
	<Src>Python_example&gt;python_out</Src>
	<Dst>Neuron&lt;&lt;&lt;neuron_in</Dst>
	<Lag>1</Lag>
</Link>
-->


<!-- END TEMPLATE -->
</xsl:template>

</xsl:stylesheet>
