from dataclasses import dataclass, field


@dataclass
class SubClass:
	name: str
	values: dict = field(default_factory=dict)


dict_template_import_one = {
	'first_dict': SubClass(
		name='First entry',
		values={
			'One': 1,
			'Two': 2,
		},
	),
}

dict_template_import_two = {
	'first_dict': SubClass(
		name='First entry',
		values={
			'One': 1,
			'Two': 2,
		},
	),
}
